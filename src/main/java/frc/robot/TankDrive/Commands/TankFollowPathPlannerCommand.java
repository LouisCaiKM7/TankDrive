package frc.robot.TankDrive.Commands;

import com.pathplanner.lib.events.Event;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TankDrive.TankSubsystem;

import java.util.*;
import java.util.function.Consumer; 
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Meters;

/**
 * Follow a PathPlanner trajectory on a tank drive.
 * It uses left/right velocity PID + optional feed-forward from the trajectory.
 */
public class TankFollowPathPlannerCommand extends Command {

    private final TankSubsystem tank;
    private final PathPlannerTrajectory trajectory;
    private final Supplier<Pose3d> robotPose;
    private final PIDController leftPid;
    private final PIDController rightPid;
    private final Distance tolerance;
    private final Consumer<Event> eventConsumer;

    private final Timer timer = new Timer();
    private final Queue<Event> eventQueue = new LinkedList<>();

    public TankFollowPathPlannerCommand(
            TankSubsystem tank,
            PathPlannerTrajectory trajectory,
            Supplier<Pose3d> robotPose,
            PIDController leftPid,
            PIDController rightPid,
            Distance tolerance,
            Consumer<Event> eventConsumer) {

        this.tank = tank;
        this.trajectory = trajectory;
        this.robotPose = robotPose;
        this.leftPid = leftPid;
        this.rightPid = rightPid;
        this.tolerance = tolerance;
        this.eventConsumer = eventConsumer;

        // Sort events by timestamp for easy polling
        List<Event> events = trajectory.getEvents();
        events.sort(Comparator.comparingDouble(Event::getTimestampSeconds));
        eventQueue.addAll(events);

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
        PathPlannerTrajectoryState state = trajectory.sample(t);

        // Feed-forward chassis speeds
        ChassisSpeeds ffSpeeds = state.fieldSpeeds;
        DifferentialDriveWheelSpeeds ffWheels = tank.getKinematics().toWheelSpeeds(ffSpeeds);

        // Current wheel speeds
        DifferentialDriveWheelSpeeds now = tank.getWheelSpeeds();

        // PID feedback on velocity
        double leftFB  = leftPid.calculate(now.leftMetersPerSecond,  ffWheels.leftMetersPerSecond);
        double rightFB = rightPid.calculate(now.rightMetersPerSecond, ffWheels.rightMetersPerSecond);

        // Combine
        DifferentialDriveWheelSpeeds cmd = new DifferentialDriveWheelSpeeds(
                ffWheels.leftMetersPerSecond + leftFB,
                ffWheels.rightMetersPerSecond + rightFB);

        tank.runVelocity(cmd);

        // Fire events
        while (eventConsumer != null &&
               !eventQueue.isEmpty() &&
               eventQueue.peek().getTimestampSeconds() <= t) {
            eventConsumer.accept(eventQueue.poll());
        }
    }

    @Override
    public void end(boolean interrupted) {
        tank.runStop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds()) &&
               robotPose.get().toPose2d().getTranslation()
                        .getDistance(trajectory.getEndState().pose.getTranslation())
               < tolerance.in(Meters);
    }
}