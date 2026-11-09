package igknighters.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import igknighters.Robot;
import igknighters.commands.Repulsor.obstacle;
import igknighters.commands.Repulsor.obstacleType;
import igknighters.util.log.Log;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.BooleanSupplier;

public class FieldConstants {

    public static class STEAL {
        public static final Pose3d POSITION_LEFT =
                new Pose3d(X_FIELD / 2, (Y_FIELD / 2) + 2, 0.0, new Rotation3d());
        public static final Pose3d POSITION_RIGHT =
                new Pose3d(X_FIELD / 2, (Y_FIELD / 2) - 2, 0.0, new Rotation3d());
    }

    public static class HUB {
        public static final double HEIGHT_METERS = 72.0 * Conv.INCHES_TO_METERS;
        public static final Pose2d POSITION_BLUE =
                new Pose2d(
                        181.56 * Conv.INCHES_TO_METERS,
                        158.32 * Conv.INCHES_TO_METERS,
                        new Rotation2d());
        public static final Pose3d POSE3D_BLUE =
                new Pose3d(
                        POSITION_BLUE.getX(),
                        POSITION_BLUE.getY(),
                        HEIGHT_METERS,
                        new Rotation3d());

        public static final Pose2d POSITION_RED =
                new Pose2d(X_FIELD - POSITION_BLUE.getX(), POSITION_BLUE.getY(), new Rotation2d());
        public static final Pose3d POSE3D_RED =
                new Pose3d(
                        POSITION_RED.getX(), POSITION_RED.getY(), HEIGHT_METERS, new Rotation3d());
    }

    public static class CLIMB {
        public static final Pose2d POSITION_BLUE =
                new Pose2d(
                        33.0 * Conv.INCHES_TO_METERS,
                        161 * Conv.INCHES_TO_METERS,
                        new Rotation2d(Math.PI / 2)); // made up value
        public static final Pose2d POSITION_RED =
                new Pose2d(
                        X_FIELD - POSITION_BLUE.getX(),
                        Y_FIELD - POSITION_BLUE.getY(),
                        new Rotation2d(-Math.PI / 2));
    }

    public static class PASS {
        public static final Pose3d POSITION_RIGHT_BLUE =
                new Pose3d(
                        36 * Conv.INCHES_TO_METERS,
                        36 * Conv.INCHES_TO_METERS,
                        0.0 * Conv.INCHES_TO_METERS,
                        new Rotation3d());

        public static final Pose3d POSITION_LEFT_BLUE =
                new Pose3d(2, FieldConstants.Y_FIELD - 2, 0.0, new Rotation3d());

        public static final Pose3d POSITION_RIGHT_RED =
                new Pose3d(FieldConstants.X_FIELD - 2, 2, 0.0, new Rotation3d());

        public static final Pose3d POSITION_LEFT_RED =
                new Pose3d(
                        FieldConstants.X_FIELD - 2,
                        FieldConstants.Y_FIELD - 2,
                        0.0,
                        new Rotation3d());
    }

    public static class OBSTACLES {
        public static final obstacle HUB_BLUE =
                new obstacle(
                        new Pose2d(
                                182.11 * Conv.INCHES_TO_METERS,
                                158.84 * Conv.INCHES_TO_METERS,
                                new Rotation2d()),
                        1.8,
                        0,
                        0,
                        obstacleType.CIRCLE);
        public static final obstacle HUB_RED =
                new obstacle(
                        new Pose2d(
                                X_FIELD - 182.11 * Conv.INCHES_TO_METERS,
                                158.84 * Conv.INCHES_TO_METERS,
                                new Rotation2d()),
                        1.8,
                        0,
                        0,
                        obstacleType.CIRCLE);
        public static final obstacle BOTTOM_BUMP_BLUE =
                new obstacle(
                        new Pose2d(
                                182.11 * Conv.INCHES_TO_METERS,
                                92.85 * Conv.INCHES_TO_METERS,
                                new Rotation2d()),
                        1.45,
                        0,
                        0,
                        obstacleType.CIRCLE);
        public static final obstacle BOTTOM_BUMP_RED =
                new obstacle(
                        new Pose2d(
                                X_FIELD - 182.11 * Conv.INCHES_TO_METERS,
                                92.85 * Conv.INCHES_TO_METERS,
                                new Rotation2d()),
                        1.45,
                        0,
                        0,
                        obstacleType.CIRCLE);
        public static final obstacle TOP_BUMP_BLUE =
                new obstacle(
                        new Pose2d(
                                182.11 * Conv.INCHES_TO_METERS,
                                Y_FIELD - 92.85 * Conv.INCHES_TO_METERS,
                                new Rotation2d()),
                        1.45,
                        0,
                        0,
                        obstacleType.CIRCLE);
        public static final obstacle TOP_BUMP_RED =
                new obstacle(
                        new Pose2d(
                                X_FIELD - 182.11 * Conv.INCHES_TO_METERS,
                                Y_FIELD - 92.85 * Conv.INCHES_TO_METERS,
                                new Rotation2d()),
                        1.45,
                        0,
                        0,
                        obstacleType.CIRCLE);
        public static final obstacle BELOW_BLUE_BUMP =
                new obstacle(
                        new Pose2d(
                                182.11 * Conv.INCHES_TO_METERS,
                                25.175 * Conv.INCHES_TO_METERS,
                                new Rotation2d()),
                        1,
                        22.2 * Conv.INCHES_TO_METERS,
                        25.175 * Conv.INCHES_TO_METERS,
                        obstacleType.SAFE_ZONE);
        public static final obstacle BELOW_RED_BUMP =
                new obstacle(
                        new Pose2d(
                                X_FIELD - 182.11 * Conv.INCHES_TO_METERS,
                                25.175 * Conv.INCHES_TO_METERS,
                                new Rotation2d()),
                        1,
                        (22.2 + 10) * Conv.INCHES_TO_METERS,
                        25.175 / 4 * Conv.INCHES_TO_METERS,
                        obstacleType.SAFE_ZONE);
        public static final obstacle ABOVE_BLUE_BUMP =
                new obstacle(
                        new Pose2d(
                                182.11 * Conv.INCHES_TO_METERS,
                                Y_FIELD - 25.295 * Conv.INCHES_TO_METERS,
                                new Rotation2d()),
                        1,
                        (22.2 + 10) * Conv.INCHES_TO_METERS,
                        25.295 / 4 * Conv.INCHES_TO_METERS,
                        obstacleType.SAFE_ZONE);
        public static final obstacle ABOVE_RED_BUMP =
                new obstacle(
                        new Pose2d(
                                X_FIELD - 182.11 * Conv.INCHES_TO_METERS,
                                Y_FIELD - 25.295 * Conv.INCHES_TO_METERS,
                                new Rotation2d()),
                        1,
                        22.2 * Conv.INCHES_TO_METERS,
                        25.295 * Conv.INCHES_TO_METERS,
                        obstacleType.SAFE_ZONE);
        public static final ArrayList<obstacle> ALL_OBSTACLES =
                new ArrayList<>(
                        Arrays.asList(
                                HUB_BLUE,
                                HUB_RED,
                                TOP_BUMP_BLUE,
                                TOP_BUMP_RED,
                                BOTTOM_BUMP_BLUE,
                                BOTTOM_BUMP_RED,
                                BELOW_BLUE_BUMP,
                                ABOVE_BLUE_BUMP,
                                BELOW_RED_BUMP,
                                ABOVE_RED_BUMP));
    }

    public static final double Y_FIELD = 316.64 * Conv.INCHES_TO_METERS; // meters
    public static final double X_FIELD = 651.12 * Conv.INCHES_TO_METERS; // meters
    public static final double ALIANCE_ZONE_BLUE = 181.56 * Conv.INCHES_TO_METERS; // meters
    public static final double ALIANCE_ZONE_RED = X_FIELD - ALIANCE_ZONE_BLUE;

    public static class TRENCH {

        public static final double TRENCH_1_X_METERS = 182.11 * Conv.INCHES_TO_METERS;
        public static final double TRENCH_2_X_METERS = 468.89 * Conv.INCHES_TO_METERS; // 13.4

        // all trench protection commands are fine to use if on bump to so no need to check for the
        // y cord

        public static boolean isOnTrench(Pose2d pose, double distanceFromCenter) {
            double x = pose.getX();
            return (x >= TRENCH_1_X_METERS - distanceFromCenter
                            && x <= TRENCH_1_X_METERS + distanceFromCenter)
                    || (x >= TRENCH_2_X_METERS - distanceFromCenter
                            && x <= TRENCH_2_X_METERS + distanceFromCenter);
        }

        public static BooleanSupplier isOnTrenchSupplier(Pose2d pose, double distanceFromCenter) {
            return () -> isOnTrench(pose, distanceFromCenter);
        }
    }

    public static class BUMP {

        public static final double HALF_X_METERS =
                (15.0 + (16 * Math.sqrt(2)))
                        * Conv.INCHES_TO_METERS; // adding the distance from center to corner
        public static final double HALF_BUMP_X_METERS = 15 * Conv.INCHES_TO_METERS;
        public static final double HALF_Y_METERS = 109 * Conv.INCHES_TO_METERS;

        public static final double BUMP_1_X_METERS = 182.11 * Conv.INCHES_TO_METERS;
        public static final double BUMP_2_X_METERS = 468.89 * Conv.INCHES_TO_METERS;

        public static final double BUMP_1_Y_METERS = 158.32 * Conv.INCHES_TO_METERS;
        public static final double BUMP_2_Y_METERS = 158.32 * Conv.INCHES_TO_METERS;

        public static boolean isInside(Pose2d pose) {
            double x = pose.getX();
            double y = pose.getY();

            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/Commands/BumpProtection: x;", x);
                Log.log("ROBOT/Commands/BumpProtection: y;", y);
            }

            // Bump 1
            if (x >= BUMP_1_X_METERS - HALF_X_METERS && x <= BUMP_1_X_METERS + HALF_X_METERS) {
                if (y >= BUMP_1_Y_METERS - HALF_Y_METERS && y <= BUMP_1_Y_METERS + HALF_Y_METERS) {
                    if (!Robot.consts.disableAllLogs()) {
                        Log.log("ROBOT/Commands/BumpProtection: inside bump 1", true);
                    }
                    return true;
                }
            }

            // Bump 2
            if (x >= BUMP_2_X_METERS - HALF_X_METERS && x <= BUMP_2_X_METERS + HALF_X_METERS) {
                if (y >= BUMP_2_Y_METERS - HALF_Y_METERS && y <= BUMP_2_Y_METERS + HALF_Y_METERS) {
                    if (!Robot.consts.disableAllLogs()) {
                        Log.log("ROBOT/Commands/BumpProtection: inside bump 2", true);
                    }
                    return true;
                }
            }

            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/Commands/BumpProtection: inside bump 2", false);
                Log.log("ROBOT/Commands/BumpProtection: inside bump 1", false);
            }

            return false;
        }
    }
}
