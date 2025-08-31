package frc.robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.TankDrive.TankCommands;
import frc.robot.TankDrive.TankConfigs;
import frc.robot.TankDrive.TankLimits;
import frc.robot.TankDrive.TankSubsystem;
import frc.robot.TankDrive.Side.Side;
import frc.robot.TankDrive.Side.SideIO;
import frc.robot.TankDrive.Side.SideIOReal;
import frc.robot.TankDrive.Side.SideIOSim;
import frc.robot.TankDrive.Imu.ImuIOPigeon2;
import frc.robot.TankDrive.Imu.ImuIOSim;
import frc.robot.TankDrive.TankConfigs.SideConfigsSim;
import frc.robot.TankDrive.TankSubsystem.TankDrivingStates;

import java.util.ArrayList;
import java.util.Optional;

import static edu.wpi.first.units.Units.*;

/** Robot container that works in simulation without hardware. */
public class RobotContainer {
    /* ---------- Joysticks ---------- */
    private final CommandXboxController controller = new CommandXboxController(0);
    private TankSubsystem tankSubsystem;
    private Side left;
    private Side right;

    private void configureSubsystems() {
        if(Robot.isReal()){
            tankSubsystem = new TankSubsystem(TankConstants.tankConfigsReal,
             TankDrivingStates.PURE_DRIVE,
             new  Pair<>(new SideIOReal(
                TankConstants.leftReal, "Left"),
                 new SideIOReal(TankConstants.rightReal, "Right")),
                 new ImuIOPigeon2(TankConstants.leftReal) );
        }else{
            tankSubsystem = new TankSubsystem(TankConstants.tankConfigsSim,
             TankDrivingStates.PURE_DRIVE,
             new  Pair<>(new SideIOSim(
                TankConstants.leftSim, "Left"),
                 new SideIOSim(TankConstants.rightSim, "Right")),
                 new ImuIOSim() );
        }
    }

    public RobotContainer() {
        configureSubsystems();
        configureBindings();
    }

    private void configureBindings() {
        tankSubsystem.setDefaultCommand(
                TankCommands.driveWithJoystick(
                        tankSubsystem,
                        /* forward/back */ () -> -controller.getLeftY(),   // -1..1
                        /* turn         */ () ->  controller.getRightX(), // -1..1
                        tankSubsystem.getTankLimit().maxLinearVelocity(),  // max speed
                        RadiansPerSecond.of(4.0),                            // max turn
                        MetersPerSecond.of(0.01),                          // deadband
                        RadiansPerSecond.of(0.01))
        );

        /* "A" button stops the robot */
        controller.a().whileTrue(TankCommands.stop(tankSubsystem));
    }

    /* ---------- Auto placeholder ---------- */
    public Command getAutonomousCommand() {
        return Commands.none();
    }
}