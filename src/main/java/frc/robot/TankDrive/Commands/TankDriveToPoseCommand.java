package frc.robot.TankDrive.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TankDrive.TankSubsystem;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

/**
 * Closed-loop PID command that drives the robot to a desired pose.
 */
public class TankDriveToPoseCommand extends Command {

    private final TankSubsystem tank;
    private final Supplier<Pose3d> robotPose;
    private final Supplier<Pose3d> targetPose;
    private final PIDController xController;
    private final PIDController thetaController;
    private final Distance translationTolerance;
    private final Angle rotationTolerance;

    public TankDriveToPoseCommand(
            TankSubsystem tank,
            Supplier<Pose3d> robotPose,
            Supplier<Pose3d> targetPose,
            PIDController xController,
            PIDController thetaController,
            Distance translationTolerance,
            Angle rotationTolerance
    ) {
        this.tank = tank;
        this.robotPose = robotPose;
        this.targetPose = targetPose;
        this.xController = xController;
        this.thetaController = thetaController;
        this.translationTolerance = translationTolerance;
        this.rotationTolerance = rotationTolerance;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(tank);
    }

    @Override
    public void initialize() {
        xController.reset();
        thetaController.reset();
    }

    @Override
    public void execute() {
        Pose3d current = robotPose.get();
        Pose3d goal    = targetPose.get();

        // error vectors in field frame
        double dx = goal.getX() - current.getX();
        double dy = goal.getY() - current.getY();
        double distError = Math.hypot(dx, dy);
        double angleError = goal.getRotation().toRotation2d().minus(current.getRotation().toRotation2d()).getRadians();

        // PID outputs (simple P for now)
        double linVel = xController.calculate(distError, 0);
        double angVel = thetaController.calculate(angleError, 0);

        // convert to wheel speeds
        ChassisSpeeds speeds = new ChassisSpeeds(linVel, 0, angVel);
        DifferentialDriveWheelSpeeds wheelSpeeds = tank.getKinematics().toWheelSpeeds(speeds);
        tank.runVelocity(wheelSpeeds);
    }

    @Override
    public boolean isFinished() {
        Pose3d current = robotPose.get();
        Pose3d goal    = targetPose.get();

        double dist = current.getTranslation().getDistance(goal.getTranslation());
        double angle = Math.abs(current.getRotation().toRotation2d().minus(goal.getRotation().toRotation2d()).getRadians());
        return dist < translationTolerance.in(Meters) && angle < rotationTolerance.in(Radians);
    }

    @Override
    public void end(boolean interrupted) {
        tank.runStop();
    }
}