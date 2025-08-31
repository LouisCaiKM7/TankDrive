package frc.robot.TankDrive;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import lombok.Builder;
import lombok.Getter;
import lombok.Setter;

/**
 * 254-style set-point generator for a differential (tank) drive.
 *
 * <p>
 * Given the previous wheel speeds and a newly-requested wheel speed,
 * produces a new set-point that obeys both the maximum linear velocity
 * and the maximum linear acceleration of each side.
 * </p>
 */
@Builder
public class TankSetPointGenerator {
    @Getter @Setter
    private TankLimits limits;

    public TankSetPoint generate(DifferentialDriveWheelSpeeds desiredSpeeds,
                                 TankSetPoint prevSetpoint,
                                 double dt) {
        DifferentialDriveWheelSpeeds prev = prevSetpoint.wheelSpeeds();

        // Apply chassis-level limits (max velocity & max acceleration)
        var result = limits.apply(prev, desiredSpeeds, dt);
        DifferentialDriveWheelSpeeds limitedSpeeds = result.getFirst();
        var accelLimits = result.getSecond();   // left & right max |a|

        // Clamp each side so that the change in speed from the previous set-point
        // does not exceed max acceleration
        double maxStepLeft = accelLimits.getFirst().magnitude() * dt;
        double maxStepRight = accelLimits.getSecond().magnitude() * dt;

        double newLeft = clampDelta(limitedSpeeds.leftMetersPerSecond,
                                    prev.leftMetersPerSecond,
                                    maxStepLeft);
        double newRight = clampDelta(limitedSpeeds.rightMetersPerSecond,
                                     prev.rightMetersPerSecond,
                                     maxStepRight);

        // Ensure the new speeds do not exceed the maximum allowed speeds
        newLeft = Math.min(newLeft, limits.maxLinearVelocity().magnitude());
        newRight = Math.min(newRight, limits.maxLinearVelocity().magnitude());

        return new TankSetPoint(new DifferentialDriveWheelSpeeds(newLeft, newRight));
    }

    private static double clampDelta(double desired, double prev, double maxDelta) {
        double delta = desired - prev;
        delta = Math.max(-maxDelta, Math.min(maxDelta, delta));
        return prev + delta;
    }
}