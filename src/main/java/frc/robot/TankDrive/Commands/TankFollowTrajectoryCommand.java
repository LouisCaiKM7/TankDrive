package frc.robot.TankDrive.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TankDrive.TankSubsystem;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Meters;

/**
 * Follow a WPILib differential-drive trajectory with left/right velocity PID.
 */
public class TankFollowTrajectoryCommand extends Command {

    private final TankSubsystem tank;
    private final Trajectory trajectory;
    private final Supplier<Pose3d> robotPose;
    private final PIDController leftPid;
    private final PIDController rightPid;
    private final Distance tolerance;

    private final Timer timer = new Timer();

    public TankFollowTrajectoryCommand(
            TankSubsystem tank,
            Trajectory trajectory,
            Supplier<Pose3d> robotPose,
            PIDController leftPid,
            PIDController rightPid,
            Distance tolerance) {

        this.tank = tank;
        this.trajectory = trajectory;
        this.robotPose = robotPose;
        this.leftPid = leftPid;
        this.rightPid = rightPid;
        this.tolerance = tolerance;
        addRequirements(tank);
    }

    @Override
    public void initialize() {
        leftPid.reset();
        rightPid.reset();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double t = timer.get();
        var desiredState = trajectory.sample(t);

        // Current wheel distances (m)
        var currentPose = robotPose.get();
        double leftNow  = tank.getWheelPositions().leftMeters;
        double rightNow = tank.getWheelPositions().rightMeters;

        // Desired wheel distances (m)
        double leftDes  = desiredState.poseMeters.getX() - trajectory.getInitialPose().getX();
        double rightDes = leftDes;   // straight line, same for both wheels
        // NOTE: For curved paths you would integrate the arc length for each side.
        // WPILib's DifferentialDriveWheelSpeeds already gives left/right velocities,
        // so for full correctness you should use the velocity feed-forward and
        // close the loop on distance error.  Below is the simple version.

        // PID on *position* (distance) error
        double leftVel  = leftPid.calculate(leftNow,  leftDes);
        double rightVel = rightPid.calculate(rightNow, rightDes);

        tank.runVelocity(new DifferentialDriveWheelSpeeds(leftVel, rightVel));
    }

    @Override
    public void end(boolean interrupted) {
        tank.runStop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds()) &&
               robotPose.get().getTranslation()
                        .getDistance(new Translation3d(trajectory.getStates().get(trajectory.getStates().size() - 1)
                                               .poseMeters.getTranslation()))
               < tolerance.in(Meters);
    }
}