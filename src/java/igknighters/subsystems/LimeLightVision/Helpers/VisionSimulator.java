package igknighters.subsystems.LimeLightVision.Helpers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import igknighters.Robot;
import igknighters.util.AprilTagLayout;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Simulates one Limelight by taking a ground-truth Pose2d, applying Gaussian noise and random
 * dropouts, and publishing it to NetworkTables exactly like a real Limelight. Vision code then
 * reads it through the same path it uses on the robot.
 */
public class VisionSimulator {
    private AprilTagLayout fieldLayout;

    // Camera constraints
    private final double minAngleRadians;
    private final double maxAngleRadians;
    private final double maxVisibleDistanceMeters;

    // Base noise (when tag is exactly 1 meter away)
    private final double baseXyStdDev;
    private final double baseThetaStdDev;
    private final double dropoutRate;

    private final List<Integer> visibleTagIds = new ArrayList<>();
    private double minDistance;
    private int heartbeat = 0;

    private final Random random = new Random();

    // ponytail: rough FOVs of the old 4 camera layout, read real mounts from limelight-configs if
    // sim accuracy ever matters
    private static final double[][] FOVS = {{60, 120}, {135, 195}, {150, 210}, {240, 300}};

    public static VisionSimulator forCameraIndex(int i) {
        double[] fov = FOVS[i % FOVS.length];
        return new VisionSimulator(fov[0], fov[1], 5, 0.0001, 0.0, 0.1);
    }

    /** Writes this frame to the Limelight's NetworkTables entries. */
    public void publish(String limelightName) {
        Pose2d pose = getEstimatedPose();
        int n = pose == null ? 0 : visibleTagIds.size();
        double[] botpose = new double[11 + 7 * n];
        if (pose != null) {
            botpose[0] = pose.getX();
            botpose[1] = pose.getY();
            botpose[5] = pose.getRotation().getDegrees();
            botpose[7] = n;
            botpose[9] = minDistance;
            for (int i = 0; i < n; i++) {
                int b = 11 + 7 * i;
                botpose[b] = visibleTagIds.get(i);
                botpose[b + 4] = minDistance;
                botpose[b + 5] = minDistance;
            }
        }
        LimelightHelpers.setLimelightNTDoubleArray(limelightName, "botpose_wpiblue", botpose);
        LimelightHelpers.setLimelightNTDoubleArray(limelightName, "botpose_orb_wpiblue", botpose);
        LimelightHelpers.setLimelightNTDouble(limelightName, "tv", n > 0 ? 1 : 0);
        LimelightHelpers.setLimelightNTDouble(limelightName, "hb", heartbeat++);
    }

    /**
     * Creates a new VisionSimulator.
     *
     * @param minAngleDegrees The left-most angle the camera can see (e.g., 45).
     * @param maxAngleDegrees The right-most angle the camera can see (e.g., -45).
     * @param maxVisibleDistanceMeters Maximum range the camera can detect a tag.
     * @param baseXyStdDev X/Y noise standard deviation at 1 meter distance.
     * @param baseThetaStdDev Rotation noise standard deviation at 1 meter distance.
     * @param dropoutRate Probability (0.0 to 1.0) of a random frame drop.
     */
    public VisionSimulator(
            double minAngleDegrees,
            double maxAngleDegrees,
            double maxVisibleDistanceMeters,
            double baseXyStdDev,
            double baseThetaStdDev,
            double dropoutRate) {

        try {
            this.fieldLayout = new AprilTagLayout();
        } catch (IOException e) {
            System.out.println("Could not load AprilTag layout");
            e.printStackTrace();
        }

        this.minAngleRadians = Math.toRadians(minAngleDegrees);
        this.maxAngleRadians = Math.toRadians(maxAngleDegrees);
        this.maxVisibleDistanceMeters = maxVisibleDistanceMeters;
        this.baseXyStdDev = baseXyStdDev;
        this.baseThetaStdDev = baseThetaStdDev;
        this.dropoutRate = Math.max(0.0, Math.min(1.0, dropoutRate));
    }

    /** Processes a ground-truth pose and returns a fuzzed estimate if a tag is visible. */
    public Pose2d getEstimatedPose() {

        visibleTagIds.clear();

        Pose2d truePose = Robot.pose_pred.getPredictedPose();
        // 1. Random hardware dropout
        if (random.nextDouble() < dropoutRate) {
            return null;
        }

        // 2. Check visibility and find the closest tag
        minDistance = Double.MAX_VALUE;
        boolean canSeeTag = false;

        for (int tag : fieldLayout.getTagPoses().keySet()) {
            Pose2d tagPose = fieldLayout.getTagPoses().get(tag).toPose2d();

            double distance = truePose.getTranslation().getDistance(tagPose.getTranslation());

            if (distance > maxVisibleDistanceMeters) {
                continue;
            }

            double angleToTag =
                    Math.atan2(tagPose.getY() - truePose.getY(), tagPose.getX() - truePose.getX());

            // Angle relative to the front of the robot, wrapped between -PI and PI
            double relativeAngle =
                    MathUtil.angleModulus(angleToTag - truePose.getRotation().getRadians());

            // 3. Check if the tag falls within our defined angle range
            boolean inFov = false;
            if (minAngleRadians < maxAngleRadians) {
                // Normal case (e.g., -45 to 45)
                inFov = relativeAngle >= minAngleRadians && relativeAngle <= maxAngleRadians;
            } else {
                // Wrap-around case (e.g., camera facing backwards, 135 to -135)
                inFov = relativeAngle >= minAngleRadians || relativeAngle <= maxAngleRadians;
            }

            if (inFov) {
                canSeeTag = true;
                visibleTagIds.add(tag);
                if (distance < minDistance) {
                    minDistance = distance;
                }
            }
        }

        if (!canSeeTag) {
            return null;
        }

        // 4. Scale noise based on distance
        double distanceMultiplier = minDistance * minDistance;
        distanceMultiplier = Math.max(1.0, distanceMultiplier);

        double appliedXyStdDev = baseXyStdDev * distanceMultiplier;
        double appliedThetaStdDev = baseThetaStdDev * distanceMultiplier;

        // 5. Apply Gaussian noise
        double fuzzedX = truePose.getX() + (random.nextGaussian() * appliedXyStdDev);
        double fuzzedY = truePose.getY() + (random.nextGaussian() * appliedXyStdDev);
        double fuzzedTheta =
                truePose.getRotation().getRadians() + (random.nextGaussian() * appliedThetaStdDev);

        return new Pose2d(new Translation2d(fuzzedX, fuzzedY), new Rotation2d(fuzzedTheta));
    }

    public List<Integer> getVisibleTagIds() {
        return visibleTagIds;
    }
}
