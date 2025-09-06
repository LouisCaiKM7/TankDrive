package frc.robot;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.TankDrive.TankConfigs;
import frc.robot.TankDrive.TankLimits;
import frc.robot.TankDrive.TankConfigs.SideConfigsReal;
import frc.robot.TankDrive.TankConfigs.SideConfigsSim;
// do not touch this file, we havn't arrived yet
import lib.ntext.NTParameter;

public class TankConstants {
    @NTParameter(tableName = "Params/Tank")
    public static final class TankParams {
        public static final double kP = 0.225;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kA = 0.0;
        public static final double kV = 0.01;
        public static final double kS = 0.0;
    }
    @NTParameter(tableName = "Params/TankTranslationPID")
    public static final class AimToPoseTranslationPID{
        public static final double kP = 2.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }
    @NTParameter(tableName = "Params/TankRotationPID")
    public static final class AimToPoseRotationPID{
        public static final double kP = 2.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }
    public static final SideConfigsReal leftReal = SideConfigsReal.builder()
        .name("Left")
        .deltaTimeInSeconds(0.02)
        .gearRatio(10.71)
        .canivoreCanBusName("6941Canivore0")
        .driveStatorCurrentLimit(Amps.of(40))
        .wheelDiameter(Inches.of(6))
        .motorInverted(new ArrayList<>(java.util.List.of(true, false)))
        .motors(new ArrayList<>(java.util.List.of(
                new TalonFX(1, "6941Canivore0"),
                new TalonFX(2, "6941Canivore0"))))
        .motorId(new ArrayList<>(java.util.List.of(1, 2)))         // <-- add if you need a gyro on this side
        .odometryFrequency(Hertz.of(100))
        .pigeonId(10)   
        .build();
    public static final SideConfigsReal rightReal = SideConfigsReal.builder()
        .name("Right")
        .deltaTimeInSeconds(0.02)
        .gearRatio(10.71)
        .canivoreCanBusName("6941Canivore0")
        .driveStatorCurrentLimit(Amps.of(40))
        .wheelDiameter(Inches.of(6))
        .motorInverted(new ArrayList<>(java.util.List.of(false, true)))
        .motors(new ArrayList<>(java.util.List.of(
                new TalonFX(3, "6941Canivore0"),
                new TalonFX(4, "6941Canivore0"))))
        .motorId(new ArrayList<>(java.util.List.of(3, 4)))
        .pigeonId(10)           // <-- add if you need a gyro on this side
        .odometryFrequency(Hertz.of(100))
        .build();
    public static final TankConfigs tankConfigsReal = TankConfigs.builder()
        .name("m_tank")
        .sides(new Pair<TankConfigs.SideConfigs,TankConfigs.SideConfigs>(rightReal, leftReal))
        .trackWidth(0.548)
        .tankLimits(new TankLimits(MetersPerSecond.of(3.5),MetersPerSecondPerSecond.of(2)))
        .build();
    public static final SideConfigsSim leftSim = SideConfigsSim.builder()
        .name("Left")
        .deltaTimeInSeconds(0.02)
        .gearRatio(10.71)
        .momentOfInertia(new ArrayList<>(List.of(KilogramSquareMeters.of(0.04), KilogramSquareMeters.of(0.04))))
        .motors(new ArrayList<>(List.of(
            DCMotor.getKrakenX60(1),
            DCMotor.getKrakenX60(1)
        )))
        .motorInverted(new ArrayList<>(List.of(true, false)))
        .driveStdDevPos(0.002)
        .driveStdDevVel(0.002)
        .wheelDiameter(Inches.of(6))
        .distanceRelativeToCenter(Inches.of(21.589/2))
        .build();
    public static final SideConfigsSim rightSim = SideConfigsSim.builder()
        .name("Right")
        .deltaTimeInSeconds(0.02)
        .gearRatio(10.71)
        .momentOfInertia(new ArrayList<>(List.of(KilogramSquareMeters.of(0.04), KilogramSquareMeters.of(0.04))))
        .motors(new ArrayList<>(List.of(
            DCMotor.getKrakenX60(1),
            DCMotor.getKrakenX60(1)
        )))
        .motorInverted(new ArrayList<>(List.of(false, true)))
        .driveStdDevPos(0.002)
        .driveStdDevVel(0.002)
        .wheelDiameter(Inches.of(6))
        .distanceRelativeToCenter(Inches.of(21.589/2))
        .build();
    public static final TankConfigs tankConfigsSim = TankConfigs.builder()
        .name("m_tank")
        .sides(new Pair<TankConfigs.SideConfigs,TankConfigs.SideConfigs>(rightSim, leftSim))
        .trackWidth(0.548)
        .tankLimits(new TankLimits(MetersPerSecond.of(4.5),MetersPerSecondPerSecond.of(2)))
        .build();
}
