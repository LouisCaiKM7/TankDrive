package frc.robot.TankDrive.Imu;
//Finished, if anychange needed, please contact, rather than just editing the file.
import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.ArrayDeque;
import java.util.Queue;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.TankDrive.TankConfigs.SideConfigsReal;

public class ImuIOPigeon1 implements ImuIO{
    private final PigeonIMU pigeon;

    private final Timer timer = new Timer();

    private final SideConfigsReal configs;

    // Status signals
        private Double yaw;
        private Double yawVelocity;
        private Double pitch;
        private Double pitchVelocity;
        private Double roll;
        private Double rollVelocity;
        
        // Odometry queues (only for yaw since that's what's needed for pose estimation)
        private Queue<Double> yawPositionQueue;
        private Queue<Double> timestampQueue;
    
        public ImuIOPigeon1(SideConfigsReal configs){
            timer.reset();
            timer.start();
            this.configs = configs;
            
            System.out.println("ImuIOPigeon: Initializing Pigeon2 with ID " + configs.pigeonId);
    
            pigeon = new PigeonIMU(configs.pigeonId);
    
            
    
            yaw = pigeon.getYaw();
            yawVelocity = 0.0;
            pitch = pigeon.getPitch();
            pitchVelocity = 0.0;
            roll = pigeon.getRoll();
            rollVelocity = 0.0;
    
            pigeon.clearStickyFaults();
            
            // Fallback queues if sync thread registration fails
            if (yawPositionQueue == null) {
                yawPositionQueue = new ArrayDeque<>();
            }
            if (timestampQueue == null) {
                timestampQueue = new ArrayDeque<>();
            }
        }
    
        @Override
        public void updateInputs(ImuIOInputs inputs) {
            yaw = pigeon.getYaw();
            yawVelocity = getYawRate(configs.deltaTimeInSeconds);
            pitch = pigeon.getPitch();
            pitchVelocity = getPitchRate(configs.deltaTimeInSeconds);
            roll = pigeon.getRoll();
            rollVelocity = getRollRate(configs.deltaTimeInSeconds);

            inputs.connected = !(yaw == null || pitch == null || roll == null);

            // Current positions and velocities
            inputs.yawPosition = Rotation2d.fromDegrees(yaw);
            inputs.yawVelocityRadPerSec = DegreesPerSecond.of(yawVelocity).in(Units.RadiansPerSecond);
            inputs.pitchPosition = Rotation2d.fromDegrees(pitch);
            inputs.pitchVelocityRadPerSec = DegreesPerSecond.of(pitchVelocity).in(Units.RadiansPerSecond);
            inputs.rollPosition = Rotation2d.fromDegrees(roll);
            inputs.rollVelocityRadPerSec = DegreesPerSecond.of(rollVelocity).in(Units.RadiansPerSecond);
            
            // Process odometry queues (same pattern as swerve modules)
            if (!yawPositionQueue.isEmpty() && !timestampQueue.isEmpty()) {
                // Convert queue data to arrays
                int queueSize = Math.min(yawPositionQueue.size(), timestampQueue.size());
                
                inputs.odometryYawTimestamps = new double[queueSize];
                inputs.odometryYawPositions = new Rotation2d[queueSize];
                inputs.odometryRotations = new Rotation3d[queueSize];
                
                for (int i = 0; i < queueSize; i++) {
                    double timestamp = timestampQueue.poll();
                    double yawDegrees = yawPositionQueue.poll();
                    
                    inputs.odometryYawTimestamps[i] = timestamp;
                    inputs.odometryYawPositions[i] = Rotation2d.fromDegrees(yawDegrees);
                    inputs.odometryRotations[i] = new Rotation3d(
                        inputs.rollPosition.getRadians(),
                        inputs.pitchPosition.getRadians(),
                        Rotation2d.fromDegrees(yawDegrees).getRadians()
                    );
                }
                
                // Clear any remaining queue items to stay synchronized
                yawPositionQueue.clear();
                timestampQueue.clear();
            } else {
                // Fallback to current values if no queue data
                inputs.odometryYawTimestamps = new double[]{timer.get()};
                inputs.odometryYawPositions = new Rotation2d[]{inputs.yawPosition};
                inputs.odometryRotations = new Rotation3d[]{
                    new Rotation3d(
                        inputs.rollPosition.getRadians(),
                        inputs.pitchPosition.getRadians(),
                        inputs.yawPosition.getRadians()
                    )
                };
            }
    }

    private final double getYawRate(double dt){
        double lastYaw = pigeon.getYaw();
        Timer.delay(configs.deltaTimeInSeconds);
        double currentYaw = pigeon.getYaw();
        double yawRate = (currentYaw - lastYaw) / dt;
        return yawRate;
    }
    private final double getPitchRate(double dt){
        double lastPitch = pigeon.getPitch();
        Timer.delay(configs.deltaTimeInSeconds);
        double currentPitch = pigeon.getPitch();
        double pitchRate = (currentPitch - lastPitch) / dt;
        return pitchRate;
    }
    private final double getRollRate(double dt){
        double lastRoll = pigeon.getRoll();
        Timer.delay(configs.deltaTimeInSeconds);
        double currentRoll = pigeon.getRoll();
        double rollRate = (currentRoll - lastRoll) / dt;
        return rollRate;
    }


}
