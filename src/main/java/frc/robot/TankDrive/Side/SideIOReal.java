package frc.robot.TankDrive.Side;
//Finished, if anychange needed, please contact, rather than just editing the file.
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volt;

import java.util.ArrayList;
import java.util.Queue;
import java.util.concurrent.locks.ReentrantLock;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.TankDrive.TankConfigs.SideConfigsReal;
import frc.utils.PhoenixSynchronizationThread;
import frc.utils.PhoenixUtils;
import lombok.Getter;

public class SideIOReal implements SideIO {
    @Getter
    private static PhoenixSynchronizationThread syncThread;

    @Getter
    private static final ReentrantLock syncLock = new ReentrantLock();

    private final ArrayList<TalonFX> motors;
    private final ArrayList<TalonFXConfiguration> talonFxConfigs;

    private ArrayList<StatusSignal<Angle>> drivePosition;
    private ArrayList<Queue<Double>> drivePositionQueue;
    private ArrayList<StatusSignal<AngularVelocity>> driveVelocity;
    private ArrayList<StatusSignal<Voltage>> driveVoltage;
    private ArrayList<StatusSignal<Current>> driveSupplyCurrentAmps;
    private ArrayList<StatusSignal<Current>> driveTorqueCurrentAmps;
    private ArrayList<StatusSignal<Temperature>> driveTemperatureCel;

    private final VelocityTorqueCurrentFOC driveVelocityRequest = new VelocityTorqueCurrentFOC(0);
    private final VoltageOut driveVoltageRequest = new VoltageOut(0);


    private final SideConfigsReal sideConfigs;

    public SideIOReal(SideConfigsReal sideConfigs, String name) {
        this.sideConfigs = sideConfigs;
        motors = new ArrayList<>(sideConfigs.motorNumber());
        talonFxConfigs = new ArrayList<>(sideConfigs.motorNumber());
        drivePosition = new ArrayList<>(sideConfigs.motorNumber());
        drivePositionQueue = new ArrayList<>(sideConfigs.motorNumber());
        driveVelocity = new ArrayList<>(sideConfigs.motorNumber());
        driveVoltage = new ArrayList<>(sideConfigs.motorNumber());
        driveSupplyCurrentAmps = new ArrayList<>(sideConfigs.motorNumber());
        driveTorqueCurrentAmps = new ArrayList<>(sideConfigs.motorNumber());
        driveTemperatureCel = new ArrayList<>(sideConfigs.motorNumber());

        configureMotors();

        for (int i = 0; i < sideConfigs.motorNumber(); i++) {
            sideConfigs.motors.set(i, new TalonFX(sideConfigs.motorId.get(i), sideConfigs.canivoreCanBusName));
            motors.add(sideConfigs.motors.get(i));
        }
        for (int i = 0; i < sideConfigs.motorNumber(); i++) {
            talonFxConfigs.set(i, new TalonFXConfiguration());
        }
        if (syncThread == null)
            syncThread = new PhoenixSynchronizationThread(syncLock, sideConfigs.odometryFrequency);
    }

    public static void startSyncThread() {
        if (syncThread != null && !syncThread.isAlive())
            syncThread.start();
    }

    


    private void configureMotors(){
        for (int i = 0; i < sideConfigs.motorNumber(); i++) {
            int finalI = i; // new effectively final variable
    
            talonFxConfigs.get(finalI).MotorOutput.Inverted = 
            sideConfigs.motorInverted.get(i) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
            talonFxConfigs.get(finalI).CurrentLimits.StatorCurrentLimitEnable = true;
            talonFxConfigs.get(finalI).CurrentLimits.StatorCurrentLimit = sideConfigs.driveStatorCurrentLimit.in(Amps);
            talonFxConfigs.get(finalI).MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
            PhoenixUtils.tryUntilOk(5, ()-> motors.get(finalI).getConfigurator().apply(talonFxConfigs.get(finalI)));
            motors.get(i).getConfigurator().apply(talonFxConfigs.get(i));
            motors.get(i).optimizeBusUtilization();

            drivePosition.set(i, motors.get(i).getPosition());
            driveVelocity.set(i, motors.get(i).getVelocity());
            driveVoltage.set(i, motors.get(i).getMotorVoltage());
            driveSupplyCurrentAmps.set(i, motors.get(i).getSupplyCurrent());
            driveTorqueCurrentAmps.set(i, motors.get(i).getStatorCurrent());
            driveTemperatureCel.set(i, motors.get(i).getDeviceTemp());

            drivePosition.get(i).setUpdateFrequency(100.0);
            driveVelocity.get(i).setUpdateFrequency(100.0);

            driveVoltage.get(i).setUpdateFrequency(50.0);
            driveSupplyCurrentAmps.get(i).setUpdateFrequency(50.0);
            driveTorqueCurrentAmps.get(i).setUpdateFrequency(50.0);

            driveTemperatureCel.get(i).setUpdateFrequency(10.0);

            if (syncThread != null && drivePosition != null) {
                drivePositionQueue.set(i, syncThread.registerSignal(drivePosition.get(i).clone()));
            }
            
        }
        if (drivePositionQueue == null) {
            drivePositionQueue = new ArrayList<>();
        }
    }

    @Override
    public void updateInputs(SideIOInputs[] inputs) {
        for(int i = 0; i < sideConfigs.motorNumber(); i++){
            inputs[i].sideMotorConnected = BaseStatusSignal.isAllGood(
                drivePosition.get(i), driveVelocity.get(i), driveVoltage.get(i),
                 driveSupplyCurrentAmps.get(i), driveTorqueCurrentAmps.get(i), driveTemperatureCel.get(i));
            inputs[i].sideMotorPositionRad = driveMotorRotationsToMechanismRad(driveVelocity.get(i).getValueAsDouble());
            inputs[i].sideMotorVelocityRadPerSec = driveMotorRotationsPerSecToMechanismRadPerSec(driveVelocity.get(i).getValueAsDouble());
            inputs[i].sideMotorTemperatureCel = driveTemperatureCel.get(i).getValueAsDouble();
            inputs[i].sideMotorVoltageVolt = driveVoltage.get(i).getValueAsDouble();
            inputs[i].sideMotorSupplyCurrentAmpere = driveSupplyCurrentAmps.get(i).getValueAsDouble();
            inputs[i].sideMotorTorqueCurrentAmpere = driveTorqueCurrentAmps.get(i).getValueAsDouble();

            if (drivePositionQueue != null && !drivePositionQueue.isEmpty()) {
                inputs[i].sideMotorPositionRadSamples = drivePositionQueue.get(i).stream().mapToDouble(
                        this::driveMotorRotationsToMechanismRad).toArray();
                drivePositionQueue.get(i).clear();
            } else {
                inputs[i].sideMotorPositionRadSamples = new double[]{inputs[i].sideMotorPositionRad};
            }
    
        }
    }
    

    @Override
    public void configSideController(double kp, double ki, double kd, double ks, double kv, double ka) {
        // Configure both PID and FF parameters in single config to avoid overwriting
        for(int i = 0; i < sideConfigs.motorNumber(); i++){
            talonFxConfigs.get(i).Slot0.kP = kp;
            talonFxConfigs.get(i).Slot0.kI = ki;
            talonFxConfigs.get(i).Slot0.kD = kd;
            talonFxConfigs.get(i).Slot0.kS = ks;
            talonFxConfigs.get(i).Slot0.kV = kv;
            talonFxConfigs.get(i).Slot0.kA = ka;
            motors.get(i).getConfigurator().apply(talonFxConfigs.get(i).Slot0);
        }
    }
    
    @Override
    public void configDriveBrake(boolean isBreak) {
        for(int i = 0; i < sideConfigs.motorNumber(); i++){
            motors.get(i).setNeutralMode(isBreak ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        }
    }

    @Override
    public void setDriveOpenLoop(Voltage voltage) {
        for(int i = 0; i < sideConfigs.motorNumber(); i++){
            motors.get(i).setControl(driveVoltageRequest.withOutput(voltage.in(Volt)));
        }
    }

    @Override
    public void setDriveVelocity(LinearVelocity des) {
        double velocityRps = linearVelocityToWheelRPS(des.in(MetersPerSecond));
        for(int i = 0; i < sideConfigs.motorNumber(); i++){
            motors.get(i).setControl(driveVelocityRequest.withVelocity(velocityRps * sideConfigs.gearRatio));
        }
    }

    @Override
    public void setDriveVelocity(LinearVelocity des, Current ff) {
        double velocityRps = linearVelocityToWheelRPS(des.in(MetersPerSecond));
        for(int i = 0; i < sideConfigs.motorNumber(); i++){
            motors.get(i).setControl(driveVelocityRequest.withVelocity(velocityRps * sideConfigs.gearRatio).withFeedForward(ff.in(Amps)));
        }
    }
    

    // ========== UNIT CONVERSION METHODS ==========

    /*
     * CONVERSION OVERVIEW:
     *
     * This tank side motors uses TalonFX motors with gear reduction for both drive and steering.
     * The interface expects wheel/mechanism units, but the motors need motor shaft units.
     *
     * Drive System:
     * - Motor shaft -> [gear ratio] -> Wheel
     * - Higher gear ratio = motor spins faster than wheel
     * - Example: 6.14:1 gear ratio means motor rotates 6.14 times per wheel rotation
     *
     */

    // ========== DRIVE MOTOR CONVERSIONS ==========

    /**
     * Convert drive motor rotations to wheel position in radians.
     * <p>
     * Flow: Motor rotations -> Wheel radians
     * Math: motor_rot * (2π rad/rot) / gear_ratio = wheel_rad
     *
     * @param motorRotations Raw motor encoder rotations
     * @return Wheel position in radians
     */
    private double driveMotorRotationsToMechanismRad(double motorRotations) {
        return Units.rotationsToRadians(motorRotations) / sideConfigs.gearRatio;
    }

    /**
     * Convert drive motor rotations per second to wheel angular velocity in rad/s.
     * <p>
     * Flow: Motor RPS -> Wheel rad/s
     * Math: motor_rps * (2π rad/rot) / gear_ratio = wheel_rad_per_sec
     *
     * @param motorRotationsPerSec Raw motor velocity in rotations per second
     * @return Wheel angular velocity in rad/s
     */
    private double driveMotorRotationsPerSecToMechanismRadPerSec(double motorRotationsPerSec) {
        return Units.rotationsToRadians(motorRotationsPerSec) / sideConfigs.gearRatio;
    }

    /**
     * Convert linear velocity to wheel rotations per second.
     * <p>
     * Flow: Linear velocity (m/s) -> Wheel RPS
     * Math: linear_vel / (wheel_diameter * π) = wheel_rps
     * <p>
     * This is used at the interface level - gear ratio is applied when commanding motors.
     *
     * @param linearVelocityMPS Linear velocity in meters per second
     * @return Wheel rotations per second
     */
    private double linearVelocityToWheelRPS(double linearVelocityMPS) {
        double wheelCircumference = sideConfigs.wheelDiameter.in(Meter) * Math.PI;
        return linearVelocityMPS / wheelCircumference;
    }

}
    

