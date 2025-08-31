package frc.robot.TankDrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.TankDrive.Commands.TankDriveToPoseCommand;
import frc.robot.TankDrive.Commands.TankFollowPathPlannerCommand;
import frc.robot.TankDrive.Commands.TankFollowTrajectoryCommand;

import java.util.function.*;
import java.util.logging.Logger;

import com.pathplanner.lib.events.Event;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.trajectory.Trajectory;

import static edu.wpi.first.units.Units.*;

/**
 * High-level command factory for the tank drive.
 */
public class TankCommands {

    /* ---------- Joystick curve helpers (mirrors swerve) ---------- */
    public static final DoubleUnaryOperator LINEAR  = x -> x;
    public static final DoubleUnaryOperator QUADRATIC = x -> x * x * Math.signum(x);
    public static final DoubleUnaryOperator CUBIC     = x -> x * x * x;

    /* ---------- 1. Drive with joystick (raw) ---------- */
    public static Command driveWithJoystick(
            TankSubsystem tank,
            DoubleSupplier linearSpeedSupplier,   // usually left Y
            DoubleSupplier angularSpeedSupplier,  // usually right X
            LinearVelocity maxLinear,
            AngularVelocity maxAngular,
            LinearVelocity linearDeadband,
            AngularVelocity angularDeadband,
            DoubleUnaryOperator linearCurve,
            DoubleUnaryOperator angularCurve
    ) {
        return Commands.run(() -> {
            
            // apply curves + deadband
            double lin = MathUtil.applyDeadband(
                    linearCurve.applyAsDouble(linearSpeedSupplier.getAsDouble()),
                    linearDeadband.in(MetersPerSecond)
            ) * maxLinear.in(MetersPerSecond);

            double ang = MathUtil.applyDeadband(
                    angularCurve.applyAsDouble(angularSpeedSupplier.getAsDouble()),
                    angularDeadband.in(RadiansPerSecond)
            ) * maxAngular.in(RadiansPerSecond);
            System.out.printf("lin=%.2f  ang=%.2f%n", lin, ang);
            System.out.println(linearSpeedSupplier.getAsDouble() +" "+ angularSpeedSupplier.getAsDouble());
            // convert to wheel speeds
            ChassisSpeeds speeds = new ChassisSpeeds(lin, 0, ang);
            DifferentialDriveWheelSpeeds wheelSpeeds =
                    tank.getKinematics().toWheelSpeeds(speeds);
            System.out.println(wheelSpeeds.leftMetersPerSecond + " " + wheelSpeeds.rightMetersPerSecond);

            tank.runVelocity(wheelSpeeds);
        }, tank);
    }

    /* ---------- 2. Convenience overload with default curves ---------- */
    public static Command driveWithJoystick(
            TankSubsystem tank,
            DoubleSupplier linearSpeedSupplier,
            DoubleSupplier angularSpeedSupplier,
            LinearVelocity maxLinear,
            AngularVelocity maxAngular,
            LinearVelocity linearDeadband,
            AngularVelocity angularDeadband
    ) {
        return driveWithJoystick(
                tank, linearSpeedSupplier, angularSpeedSupplier,
                maxLinear, maxAngular, linearDeadband, angularDeadband,
                QUADRATIC, QUADRATIC
        );
    }

    /* ---------- 3. Drive to pose ---------- */
    public static Command driveToPose(
            TankSubsystem tank,
            Supplier<Pose3d> robotPoseSupplier,
            Supplier<Pose3d> targetPoseSupplier,
            PIDController linController,
            PIDController rotController,
            Distance translationTolerance,
            Angle rotationTolerance
    ) {
        return new TankDriveToPoseCommand(
                tank, robotPoseSupplier, targetPoseSupplier,
                linController, rotController,
                translationTolerance, rotationTolerance
        );
    }

    /* ---------- 4. Stop ---------- */
    public static Command stop(TankSubsystem tank) {
        return Commands.runOnce(tank::runStop, tank);
    }

    /* ---------- 5. Reset pose ---------- */
    public static Command resetPose(TankSubsystem tank, Pose3d pose) {
        return Commands.runOnce(() -> tank.resetEstimatedPose(pose), tank);
    }

    /* ---------- 6. Follow a WPILib Trajectory ---------- */
    public static Command followTrajectory(
        TankSubsystem tank,
        Trajectory trajectory,
        Supplier<Pose3d> robotPoseSupplier,
        PIDController leftController,
        PIDController rightController,
        Distance tolerance) {

    return new TankFollowTrajectoryCommand(
            tank, trajectory, robotPoseSupplier,
            leftController, rightController, tolerance);
    }

    /* ---------- 7. Follow a PathPlanner trajectory ---------- */
    public static Command followPathPlannerPath(
        TankSubsystem tank,
        PathPlannerTrajectory trajectory,
        Supplier<Pose3d> robotPoseSupplier,
        PIDController leftController,
        PIDController rightController,
        Distance tolerance,
        Consumer<Event> eventConsumer) {

    return new TankFollowPathPlannerCommand(
            tank, trajectory, robotPoseSupplier,
            leftController, rightController, tolerance, eventConsumer);
    }

    /* ---------- 8. SysId ---------- */
    public static SysIdRoutine sysId(
            TankSubsystem tank,
            Velocity<edu.wpi.first.units.VoltageUnit> ramp,
            edu.wpi.first.units.measure.Voltage step,
            Time timeout
    ) {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        ramp, step, timeout,
                        state -> {
                            // Logging callback
                        }
                ),
                new SysIdRoutine.Mechanism(
                        tank::runVoltage,
                        null,   // no log consumer, we do it above
                        tank
                )
        );
    }

    /* ---------- Helper functional interface ---------- */
    @FunctionalInterface
    public static interface DoubleUnaryOperator {
        double applyAsDouble(double operand);
    }
}