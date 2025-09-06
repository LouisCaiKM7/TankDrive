package frc.robot.TankDrive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.TankDrive.Imu.ImuIO;
import frc.robot.TankDrive.Imu.ImuIOInputsAutoLogged;
import frc.robot.TankDrive.Side.Side;
import frc.robot.TankDrive.Side.SideIO;
import frc.utils.LoggedTracer;
import lombok.Getter;

public class TankSubsystem extends SubsystemBase {
    static final Lock odomLock = new ReentrantLock();
    private final DifferentialDriveKinematics kinematics;
    private final DifferentialDrivePoseEstimator3d poseEstimatorUsingGyro;
    private DifferentialDriveWheelPositions wheelPositions;
        private final TankPoseEstimator tankPoseEstimator;
            private final TankConfigs configs;
            private final Pair<Side, Side> sides;
            private final ImuIO imuIO;
            private final ImuIOInputsAutoLogged imuInputs;
            private final boolean useGyro;
            private MODE mode = MODE.VELOCITY;
            @Getter
            private Voltage previouslyAppliedVoltage;
            double[] timeStamps;
            
        
            public TankSubsystem(
                TankConfigs configs,
                TankDrivingStates state,
                Pair<SideIO, SideIO> sideIOs,
                ImuIO imuIO
            ) {
                this.wheelPositions = new DifferentialDriveWheelPositions(0.0d, 0.0d);
                this.configs = configs;
                this.kinematics = new DifferentialDriveKinematics(configs.trackWidth);
                this.sides = new Pair<>(
                    new Side(configs.sides.getFirst(), sideIOs.getFirst()),
                    new Side(configs.sides.getSecond(), sideIOs.getSecond())
                );
                // this.currentSetPoint = new TankSetPoint(new DifferentialDriveWheelSpeeds(0.0d, 0.0d));
                // this.setPointGenerator = TankSetPointGenerator.builder().limits(configs.tankLimits).build();
                this.useGyro = !(state == TankDrivingStates.PURE_DRIVE);
        
                if (useGyro) {
                    this.imuIO = imuIO;
                    this.imuInputs = new ImuIOInputsAutoLogged();
                    this.poseEstimatorUsingGyro = new DifferentialDrivePoseEstimator3d(
                        kinematics,
                        new Rotation3d(),
                        this.wheelPositions.leftMeters,
                        this.wheelPositions.rightMeters,
                        new Pose3d()
                    );
                    this.tankPoseEstimator = null;
                } else {
                    this.imuIO = null;
                    this.imuInputs = null;
                    this.poseEstimatorUsingGyro = null;
            
                    Matrix<N4, N1> processNoise = VecBuilder.fill(
                        0.02,  // X noise (meters)
                        0.02,  // Y noise (meters)
                        0.02,  // Z noise (meters)
                        Units.degreesToRadians(1.0) // Heading noise (radians)
                    );
                
                    this.tankPoseEstimator = new TankPoseEstimator(
                        configs.trackWidth,
                        new Pose3d(),
                        processNoise
                    );
                }
            }
        
            @Override
            public void periodic() {
                timeStamps = new double[]{Timer.getFPGATimestamp()};
                sides.getFirst().periodic();
                sides.getSecond().periodic();
                sides.getFirst().updateInputs();
                sides.getSecond().updateInputs();
                odomLock.lock();
                wheelPositions = new DifferentialDriveWheelPositions(sides.getFirst().getDriveDistance(), sides.getSecond().getDriveDistance());
            var wheelsPositionWithTime = getSampledWheelPositions();
            var now = Timer.getTimestamp();
            if (useGyro) {
                imuInputs.yawVelocityRadPerSecCmd = getChassisSpeeds().omegaRadiansPerSecond;
                imuIO.updateInputs(imuInputs);
                var rotations = imuInputs.odometryRotations;
                Logger.processInputs(configs.name + "/IMU", imuInputs);
                for(int i = 0; i < wheelsPositionWithTime.size(); i++){
                    var positionAtTime = wheelsPositionWithTime.get(i);
                    poseEstimatorUsingGyro.updateWithTime(now,
                    rotations[i], positionAtTime.getSecond());
                }
                
                Logger.recordOutput(configs.name+ "/PoseEstimatorPose", poseEstimatorUsingGyro.getEstimatedPosition());
                odomLock.unlock();
            }
            else{
                for(int i = 0; i < wheelsPositionWithTime.size(); i++){
                    var positionAtTime = wheelsPositionWithTime.get(i);
                    tankPoseEstimator.updateWithTime(now,
                    positionAtTime.getSecond().leftMeters, positionAtTime.getSecond().rightMeters);
                }
                Logger.recordOutput(configs.name+ "/PoseEstimatorPose", tankPoseEstimator.getPose());
                odomLock.unlock();
            }
    
            LoggedTracer.record(configs.name + "/INPUTS");
    
            Logger.recordOutput(configs.name + "/Mode", mode);
            Logger.recordOutput(configs.name + "/leftSpeed", getWheelSpeeds().leftMetersPerSecond);
            Logger.recordOutput(configs.name + "/rightSpeed", getWheelSpeeds().rightMetersPerSecond);

            // Logger.recordOutput(configs.name + "/desiredveloleft", currentSetPoint.wheelSpeeds().leftMetersPerSecond);
            // Logger.recordOutput(configs.name + "/desiredveloright", currentSetPoint.wheelSpeeds().rightMetersPerSecond);
        }
    
    
        public ChassisSpeeds getChassisSpeeds() {
            return kinematics.toChassisSpeeds(getWheelSpeeds());
        }
    
        public DifferentialDriveWheelSpeeds getWheelSpeeds() {
            return new DifferentialDriveWheelSpeeds(sides.getFirst().getVelocity(), sides.getSecond().getVelocity());
        }
        public DifferentialDriveWheelPositions getWheelPositions() {
            return wheelPositions;
        }
    
        public List<Pair<Double, DifferentialDriveWheelPositions>> getSampledWheelPositions() {
            double[] timestamps = this.timeStamps;
        
            // Get sampled distances from left and right sides
            Distance[] leftSamples = sides.getFirst().getSampledSideDistances();
            Distance[] rightSamples = sides.getSecond().getSampledSideDistances();
        
            // Find minimum sample size to avoid index out of bounds
            int sampleCount = Math.min(Math.min(leftSamples.length, rightSamples.length), timestamps.length);
        
            // Prepare result list
            List<Pair<Double, DifferentialDriveWheelPositions>> result = new ArrayList<>(sampleCount);
        
            // For each timestamp, create the wheel positions
            for (int i = 0; i < sampleCount; i++) {
                DifferentialDriveWheelPositions positions = new DifferentialDriveWheelPositions(
                    leftSamples[i],
                    rightSamples[i]
                );
        
                // Add timestamp + positions to result
                result.add(new Pair<>(timestamps[i], positions));
            }
        
            return result;
        }
        
        public void addVisionMeasurement(
          Pose3d visionRobotPoseMeters,
          double timestampSeconds,
          Matrix<N4, N1> visionMeasurementStdDevs) {
            if(useGyro){
                poseEstimatorUsingGyro.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
            }
            else{
                tankPoseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
            }
        }
    
        public Optional<Pose3d> getEstimatedPoseAt(Time time) {
            if (useGyro) {
                return poseEstimatorUsingGyro.sampleAt(time.in(Seconds));
            }
            else{
                return tankPoseEstimator.sampleAt(time.in(Seconds));
            }
        }
    
        public void resetEstimatedPose(Pose3d pose) {
            odomLock.lock();
            if (useGyro) {
                poseEstimatorUsingGyro.resetPosition(imuInputs.odometryRotations[0], getWheelPositions(), pose);
            }
            else{
                tankPoseEstimator.resetPose(pose, getWheelPositions().leftMeters, getWheelPositions().rightMeters);
            }
            odomLock.unlock();
        }
    
        public void runVoltage(Voltage voltage) {
            mode = MODE.VOLTAGE;
            previouslyAppliedVoltage = voltage;
            sides.getFirst().runVoltage(voltage);
            sides.getSecond().runVoltage(voltage);
        }
        public void runVelocity(DifferentialDriveWheelSpeeds velocity) {
            mode = MODE.VELOCITY;
            sides.getFirst().runVelocity(MetersPerSecond.of(configs.tankLimits.apply(velocity, velocity, 0.02).getFirst().leftMetersPerSecond));
            sides.getSecond().runVelocity(MetersPerSecond.of(configs.tankLimits.apply(velocity, velocity, 0.02).getFirst().rightMetersPerSecond));
            

            // currentSetPoint = setPointGenerator.generate(velocity, currentSetPoint, 0.02);
        
            // sides.getFirst().runVelocity(MetersPerSecond.of(currentSetPoint.wheelSpeeds().leftMetersPerSecond));
            // sides.getSecond().runVelocity(MetersPerSecond.of(currentSetPoint.wheelSpeeds().rightMetersPerSecond));
        }
        public void runVelocity(DifferentialDriveWheelSpeeds velocity, Pair<Current, Current> tou){
            mode = MODE.VELOCITY;

            sides.getFirst().runVelocity(MetersPerSecond.of(configs.tankLimits.apply(velocity, velocity, 0.02).getFirst().leftMetersPerSecond),tou.getFirst());
            sides.getSecond().runVelocity(MetersPerSecond.of(configs.tankLimits.apply(velocity, velocity, 0.02).getFirst().rightMetersPerSecond), tou.getSecond());
            

            // currentSetPoint = setPointGenerator.generate(velocity, currentSetPoint, 0.02);

            // sides.getFirst().runVelocity(MetersPerSecond.of(currentSetPoint.wheelSpeeds().leftMetersPerSecond), tou.getFirst());
            // sides.getSecond().runVelocity(MetersPerSecond.of(currentSetPoint.wheelSpeeds().rightMetersPerSecond), tou.getSecond());
            
        }

        public void runStop() {
            runVoltage(Volts.of(0.0d));
        }
    
        public TankLimits getTankLimit(){
            return configs.tankLimits;
        }

        public Pose3d getEstimatedPose() {
            return tankPoseEstimator.getPose();
        }

        public DifferentialDriveKinematics getKinematics() {
            return kinematics;
        }

    //add as much as you want
    public enum TankDrivingStates {
        PURE_DRIVE,
        WITH_PIGEON
    }

    public enum MODE {
        VELOCITY, VOLTAGE
    }
}
