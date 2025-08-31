package frc.robot.TankDrive.Side;
//Finished, if anychange needed, please contact, rather than just editing the file.

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface SideIO {
    default void updateInputs(SideIOInputs[] inputs) {
    }

    // set
    default void setDriveOpenLoop(Voltage des) {
    }
    default void setDriveVelocity(LinearVelocity des) {
    }
    default void setDriveVelocity(LinearVelocity des, Current ff) {
        setDriveVelocity(des);
    }

    // config
    default void configSideController(double kp, double ki, double kd, double ks, double kv, double ka) {
    }
    default void configDriveBrake(boolean isBreak) {
    }

    @AutoLog
    class SideIOInputs {
        public boolean sideMotorConnected;
        public double sideMotorPositionRad;
        public double[] sideMotorPositionRadSamples;
        public double sideMotorVelocityRadPerSec;
        public double sideMotorTemperatureCel;
        public double sideMotorVoltageVolt;
        public double sideMotorSupplyCurrentAmpere;
        public double sideMotorTorqueCurrentAmpere;
    }
}
