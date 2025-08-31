package frc.robot.TankDrive.Side;
//Finished, if anychange needed, please contact, rather than just editing the file.
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.ArrayList;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.TankDrive.TankConfigs.SideConfigsSim;
import frc.utils.Logging;

import static edu.wpi.first.units.Units.Volt;

public class SideIOSim implements SideIO {
    private final SideConfigsSim sideConfigs;

    private ArrayList<DCMotorSim> motorSims;
    private double[] driverMotorAppliedVoltage;

    private PIDController pidController;
    private SimpleMotorFeedforward driveFF;
    private boolean isDriveCloseLoop = false;

    public SideIOSim(SideConfigsSim sideConfigs, String name){
        this.sideConfigs = sideConfigs;

        motorSims = new ArrayList<>(sideConfigs.motorNumber());
        driverMotorAppliedVoltage = new double[sideConfigs.motorNumber()];

        // 2. fill with dummy objects so set(i, ...) works
        for (int i = 0; i < sideConfigs.motorNumber(); i++) {
            motorSims.add(null);
        }
        initializeControllers();
        initializePlants();

    }
    
    private void initializePlants() {
        for (int i = 0; i < sideConfigs.motorNumber(); i++) {
            motorSims.set(i, new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    sideConfigs.motors.get(i),
                    sideConfigs.momentOfInertia.get(i).in(KilogramSquareMeters),
                    sideConfigs.gearRatio),
                sideConfigs.motors.get(i),
                sideConfigs.driveStdDevPos,
                sideConfigs.driveStdDevVel));
        }
    }

    private void initializeControllers() {
        pidController = new PIDController(0,0,0);
        driveFF = new SimpleMotorFeedforward(0,0,0);
        pidController.reset();
    }
    
    @Override
    public void updateInputs(SideIOInputs[] inputs) {
        for (int i = 0; i < sideConfigs.motorNumber(); i++) {
            motorSims.get(i).update(sideConfigs.deltaTimeInSeconds);
            inputs[i].sideMotorConnected = true;
            inputs[i].sideMotorPositionRad = motorSims.get(i).getAngularPositionRad();
            inputs[i].sideMotorPositionRadSamples = new double[] {inputs[i].sideMotorPositionRad};
            inputs[i].sideMotorVelocityRadPerSec = motorSims.get(i).getAngularVelocityRadPerSec();
            inputs[i].sideMotorTemperatureCel = 25.0;
            inputs[i].sideMotorVoltageVolt = driverMotorAppliedVoltage[i];
            inputs[i].sideMotorSupplyCurrentAmpere = motorSims.get(i).getCurrentDrawAmps();
            inputs[i].sideMotorTorqueCurrentAmpere = 
            motorSims.get(i).getTorqueNewtonMeters() / sideConfigs.motors.get(i).KtNMPerAmp;

            motorSims.get(i).setInputVoltage(inputs[i].sideMotorVoltageVolt);
            motorSims.get(i).update(sideConfigs.deltaTimeInSeconds);
        }
    }

    @Override
    public void setDriveOpenLoop(Voltage des) {
        isDriveCloseLoop = false;
        for (int i = 0; i < sideConfigs.motorNumber(); i++) {
            driverMotorAppliedVoltage[i] = des.in(Volt);
        }
    }

    @Override
    public void setDriveVelocity(LinearVelocity des) {
        if (!isDriveCloseLoop) {
            isDriveCloseLoop = true;
            pidController.reset();
        }
        Distance distancePerRotation = sideConfigs.wheelDiameter.times(Math.PI);
        AngularVelocity angularVelocityDes = RotationsPerSecond.of(
                des.in(MetersPerSecond) / distancePerRotation.in(Meter));
        for (int i = 0; i < sideConfigs.motorNumber(); i++) {
            double fb = pidController.calculate(
                motorSims.get(i).getAngularVelocityRadPerSec() / (2 * Math.PI), angularVelocityDes.in(RotationsPerSecond));
            double ff = driveFF.calculate(angularVelocityDes.in(RotationsPerSecond));
            driverMotorAppliedVoltage[i] = fb + ff;
        }
    }

    @Override
    public void configSideController(double kp, double ki, double kd, double ks, double kv, double ka) {
        pidController.setPID(kp, ki, kd);
        driveFF = new SimpleMotorFeedforward(ks, kv, ka);
        Logging.info(
                sideConfigs.name + "Side",
                "Side drive controller updated! kp: %.2f, ki: %.2f, kd: %.2f",
                kp, ki, kd
        );
    }

}
