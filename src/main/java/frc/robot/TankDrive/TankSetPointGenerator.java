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

    /**
     * Generate the next kinematically-feasible set-point.
     *
     * @param desiredSpeeds desired wheel speeds (from driver / path follower)
     * @param prevSetpoint  previous set-point (contains already-limited speeds)
     * @param dt            loop period, seconds
     * @return next feasible set-point
     */
    public TankSetPoint generate(DifferentialDriveWheelSpeeds desiredSpeeds,
                                 TankSetPoint prevSetpoint,
                                 double dt) {

        DifferentialDriveWheelSpeeds prev = prevSetpoint.wheelSpeeds();

        /* ------------------------------------------------------------------
         * 1.  Apply chassis-level limits (max velocity & max acceleration).
         * ------------------------------------------------------------------ */
        var result = limits.apply(prev, desiredSpeeds, dt);
        DifferentialDriveWheelSpeeds limitedSpeeds = result.getFirst();
        var accelLimits = result.getSecond();   // left & right max |a|

        /* ------------------------------------------------------------------
         * 2.  Clamp each side so that the *change* in speed from the
         *     previous set-point does not exceed max acceleration.
         * ------------------------------------------------------------------ */
        double maxStepLeft  = accelLimits.getFirst().magnitude()  * dt;
        double maxStepRight = accelLimits.getSecond().magnitude() * dt;

        double newLeft  = clampDelta(limitedSpeeds.leftMetersPerSecond,
                                     prev.leftMetersPerSecond,
                                     maxStepLeft);
        double newRight = clampDelta(limitedSpeeds.rightMetersPerSecond,
                                     prev.rightMetersPerSecond,
                                     maxStepRight);

        return new TankSetPoint(
                new DifferentialDriveWheelSpeeds(newLeft, newRight));
    }

    /* ---------------------------------------------------------------------- */
    /* Helpers                                                                */
    /* ---------------------------------------------------------------------- */

    /** Clamp a speed so that |new - prev| smaller than or equals to maxDelta. */
    private static double clampDelta(double desired,
                                     double prev,
                                     double maxDelta) {
        double delta = desired - prev;
        delta = Math.max(-maxDelta, Math.min(maxDelta, delta));
        return prev + delta;
    }
}