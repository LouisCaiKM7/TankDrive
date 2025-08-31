package frc.robot.TankDrive;

import java.util.NavigableMap;
import java.util.Optional;
import java.util.TreeMap;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Twist3d;

public class TankPoseEstimator {
    private Pose3d poseMeters;
    private final double trackWidthMeters;

    // Buffer of past odometry poses for timestamp alignment
    private final NavigableMap<Double, Pose3d> odometryPoseBuffer = new TreeMap<>();

    // Kalman gain matrix for fusing vision and odometry (6x6 matrix: x,y,z,rx,ry,rz)
    private final Matrix<N4, N1> m_q;  // Process noise matrix (from drivetrain)
    private final edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N6, N6> m_visionK;

    private double prevLeftMeters = 0.0;
    private double prevRightMeters = 0.0;

    // Vision updates map: timestamp -> vision pose
    private final NavigableMap<Double, Pose3d> m_visionUpdates = new TreeMap<>();

    public TankPoseEstimator(
        double trackWidthMeters,
        Pose3d initialPose,
        Matrix<N4, N1> processNoise) {

        this.trackWidthMeters = trackWidthMeters;
        this.poseMeters = initialPose;
        this.m_q = processNoise;
        this.m_visionK = Matrix.eye(Nat.N6());
    }

    /** Resets odometry and clears buffers. */
    public void resetPose(Pose3d newPose, double leftMeters, double rightMeters) {
        poseMeters = newPose;
        prevLeftMeters = leftMeters;
        prevRightMeters = rightMeters;
        odometryPoseBuffer.clear();
    }

    /** Updates the pose using wheel distances. */
    public Pose3d updatePose(double leftMeters, double rightMeters) {
        double dLeft = leftMeters - prevLeftMeters;
        double dRight = rightMeters - prevRightMeters;

        prevLeftMeters = leftMeters;
        prevRightMeters = rightMeters;

        double dTheta = (dRight - dLeft) / trackWidthMeters;
        double dS = (dRight + dLeft) / 2.0;

        double heading = poseMeters.toPose2d().getRotation().getRadians();
        double headingMid = heading + dTheta / 2.0;

        double dx = dS * Math.cos(headingMid);
        double dy = dS * Math.sin(headingMid);

        poseMeters = new Pose3d(
            poseMeters.getX() + dx,
            poseMeters.getY() + dy,
            0,
            new Rotation3d(new Rotation2d(heading + dTheta))
        );

        // Save odometry sample into buffer with timestamp
        double now = Timer.getFPGATimestamp();
        odometryPoseBuffer.put(now, new Pose3d(
            poseMeters.getX(), poseMeters.getY(), 0,
            new Rotation3d(0, 0, poseMeters.toPose2d().getRotation().getRadians())
        ));

        return poseMeters;
    }

    /** Adds a vision measurement with default stdDevs. */
    public void addVisionMeasurement(Pose3d visionRobotPoseMeters, double timestampSeconds) {
        // Default: we trust vision at 50 cm and heading ±10 deg
        Matrix<N4, N1> defaultStdDevs = VecBuilder.fill(0.5, 0.5, 0.5, Math.toRadians(10.0));
        setVisionMeasurementStdDevs(defaultStdDevs);
        fuseVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    /**
     * Returns the pose at a given timestamp, if the buffer is not empty.
     *
     * @param timestampSeconds The pose's timestamp in seconds.
     * @return The pose at the given timestamp (or Optional.empty() if the buffer is empty).
     */
    public Optional<Pose3d> sampleAt(double timestampSeconds) {
        if (odometryPoseBuffer.isEmpty()) {
            return Optional.empty();
        }

        // Clamp the timestamp to the range of the odometry buffer
        double oldest = odometryPoseBuffer.firstKey();
        double newest = odometryPoseBuffer.lastKey();
        timestampSeconds = Math.max(oldest, Math.min(timestampSeconds, newest));

        // Get the odometry sample at the timestamp (interpolated if necessary)
        Optional<Pose3d> odometrySample = getOdometrySample(timestampSeconds);
        if (odometrySample.isEmpty()) return Optional.empty();

        // If there are no vision updates before this timestamp, return odometry only
        if (m_visionUpdates.isEmpty() || timestampSeconds < m_visionUpdates.firstKey()) {
            return odometrySample;
        }

        // Get the latest vision update before or at the timestamp
        Double floorTimestamp = m_visionUpdates.floorKey(timestampSeconds);
        Pose3d visionPose = m_visionUpdates.get(floorTimestamp);

        // Fuse vision with odometry
        Twist3d twist = odometrySample.get().log(visionPose);

        var k_times_twist = m_visionK.times(
            VecBuilder.fill(twist.dx, twist.dy, twist.dz, twist.rx, twist.ry, twist.rz)
        );

        Twist3d scaledTwist = new Twist3d(
            k_times_twist.get(0, 0),
            k_times_twist.get(1, 0),
            k_times_twist.get(2, 0),
            k_times_twist.get(3, 0),
            k_times_twist.get(4, 0),
            k_times_twist.get(5, 0)
        );

        Pose3d correctedPose = odometrySample.get().exp(scaledTwist);
        return Optional.of(correctedPose);
    }


    /** Adds a vision measurement with custom stdDevs. */
    public void addVisionMeasurement(
        Pose3d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N4, N1> visionMeasurementStdDevs) {
        setVisionMeasurementStdDevs(visionMeasurementStdDevs);
        fuseVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    /** Sets the Kalman gain matrix based on vision standard deviations. */
    public final void setVisionMeasurementStdDevs(Matrix<N4, N1> visionMeasurementStdDevs) {
        var r = new double[4];
        for (int i = 0; i < 4; ++i) {
            r[i] = visionMeasurementStdDevs.get(i, 0) * visionMeasurementStdDevs.get(i, 0);
        }

        // Solve closed-form Kalman gain for continuous Kalman filter with A=0, C=I.
        for (int row = 0; row < 4; ++row) {
            if (m_q.get(row, 0) == 0.0) {
                m_visionK.set(row, row, 0.0);
            } else {
                m_visionK.set(
                    row, row,
                    m_q.get(row, 0) /
                        (m_q.get(row, 0) + Math.sqrt(m_q.get(row, 0) * r[row]))
                );
            }
        }

        // Apply same gain for rx, ry, rz
        double angle_gain = m_visionK.get(3, 3);
        m_visionK.set(4, 4, angle_gain);
        m_visionK.set(5, 5, angle_gain);
    }

    /** Actually fuses a vision measurement into the pose estimate. */
    private void fuseVisionMeasurement(Pose3d visionRobotPoseMeters, double timestampSeconds) {
        m_visionUpdates.put(timestampSeconds, visionRobotPoseMeters);
    }

    /** Finds the closest odometry sample for given timestamp. */
    private Optional<Pose3d> getOdometrySample(double timestamp) {
        if (odometryPoseBuffer.containsKey(timestamp)) {
            return Optional.of(odometryPoseBuffer.get(timestamp));
        }
        var floor = odometryPoseBuffer.floorEntry(timestamp);
        var ceil = odometryPoseBuffer.ceilingEntry(timestamp);
        if (floor == null && ceil == null) return Optional.empty();
        if (floor == null) return Optional.of(ceil.getValue());
        if (ceil == null) return Optional.of(floor.getValue());
        return Optional.of(
            Math.abs(timestamp - floor.getKey()) < Math.abs(ceil.getKey() - timestamp)
                ? floor.getValue()
                : ceil.getValue()
        );
    }

    /** Returns current pose estimate. */
    public Pose3d getPose() {
        return poseMeters;
    }

    /**
     * Updates pose with a timestamp, fusing odometry and latest vision if available.
     */
    public Pose3d updateWithTime(double currentTimeSeconds, double leftMeters, double rightMeters) {
        // Step 1: Update odometry
        Pose3d odometryEstimate = updatePose(leftMeters, rightMeters);

        // Step 2: Add odometry sample to buffer
        odometryPoseBuffer.put(currentTimeSeconds, odometryEstimate);

        // Step 3: Fuse latest vision if available
        if (!m_visionUpdates.isEmpty()) {
            var latestVisionEntry = m_visionUpdates.lastEntry();
            Pose3d visionPose = latestVisionEntry.getValue();
            double visionTimestamp = latestVisionEntry.getKey();

            if (visionTimestamp <= currentTimeSeconds) {
                Twist3d twist = odometryEstimate.log(visionPose);
                var k_times_twist = m_visionK.times(
                    VecBuilder.fill(twist.dx, twist.dy, twist.dz, twist.rx, twist.ry, twist.rz)
                );

                Twist3d scaledTwist = new Twist3d(
                    k_times_twist.get(0, 0),
                    k_times_twist.get(1, 0),
                    k_times_twist.get(2, 0),
                    k_times_twist.get(3, 0),
                    k_times_twist.get(4, 0),
                    k_times_twist.get(5, 0)
                );

                poseMeters = odometryEstimate.exp(scaledTwist);
            } else {
                poseMeters = odometryEstimate;
            }
        } else {
            poseMeters = odometryEstimate;
        }

        return poseMeters;
    }
}
