package igknighters.constants.Wayfinder;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import igknighters.constants.Conv;
import java.util.stream.Stream;
import monologue.ProceduralStructGenerator;
import wayfinder.repulsorField.Obstacle;
import wayfinder.repulsorField.Obstacle.HorizontalObstacle;
import wayfinder.repulsorField.Obstacle.VerticalObstacle;
import wpilibExt.AllianceSymmetry;

public class WAYFINDERFIELD2025 {
    static double FIELD_LENGTH = 57.6 * Conv.FEET_TO_METERS;
    static double FIELD_WIDTH = 26.0 * Conv.FEET_TO_METERS;
    private static final Translation2d[] REEF_VERTICES_LOC =
            new Translation2d[] {
                new Translation2d(3.658, 3.546),
                new Translation2d(3.658, 4.506),
                new Translation2d(4.489, 4.987),
                new Translation2d(5.3213, 4.506),
                new Translation2d(5.3213, 3.546),
                new Translation2d(FIELD_LENGTH - 4.489, 3.065),
                new Translation2d(FIELD_LENGTH - 3.658, 3.546),
                new Translation2d(FIELD_LENGTH - 3.658, 4.506),
                new Translation2d(FIELD_LENGTH - 4.489, 4.987),
                new Translation2d(FIELD_LENGTH - 5.3213, 4.506),
                new Translation2d(FIELD_LENGTH - 5.3213, 3.546),
                new Translation2d(FIELD_LENGTH - 4.489, 3.065)
            };

    private static Translation2d inBetween(Translation2d a, Translation2d b) {
        return a.interpolate(b, 0.5);
    }

    private static final Obstacle[] REEF_VERTICES;

    static {
        REEF_VERTICES = new Obstacle[REEF_VERTICES_LOC.length];
        for (int i = 0; i < REEF_VERTICES_LOC.length; i++) {
            REEF_VERTICES[i] =
                    new Obstacle.SnowmanObstacle(REEF_VERTICES_LOC[i], 3.0, .23, 0.3, 0.8, 0.4);
        }
    }

    private static Obstacle[] reefSides(int ignoreSide) {
        Obstacle[] sides = new Obstacle[REEF_VERTICES.length];
        for (int i = 0; i < REEF_VERTICES.length / 2; i++) {
            Translation2d blueInbetween =
                    inBetween(REEF_VERTICES_LOC[i], REEF_VERTICES_LOC[(i + 1) % 6]);
            Translation2d redInbetween =
                    inBetween(REEF_VERTICES_LOC[i + 6], REEF_VERTICES_LOC[(i + 1) % 6 + 6]);
            if (i == ignoreSide) {
                sides[i] =
                        new Obstacle(0.0, true) {
                            @Override
                            public Translation2d getForceAtPosition(
                                    Translation2d position, Translation2d goal) {
                                return Translation2d.kZero;
                            }

                            @Override
                            protected double distToForceMag(double dist, double maxRange) {
                                return 0.0;
                            }
                        };
                sides[i + 6] = sides[i];
            } else {
                sides[i] = new Obstacle.SnowmanObstacle(blueInbetween, 3.0, 1.0, 1.0, 0.05, 0.05);
                sides[i + 6] =
                        new Obstacle.SnowmanObstacle(redInbetween, 3.0, 1.0, 1.0, 0.05, 0.05);
            }
        }
        return sides;
    }

    private static final Obstacle[] REEF_SMALL = {
        new Obstacle.TeardropObstacle(new Translation2d(4.49, 4), 2.0, 1.45, 1.0, 4.0, 2.2),
        new Obstacle.TeardropObstacle(new Translation2d(13.08, 4), 2.0, 1.45, 1.0, 4.0, 2.2)
    };

    private static final Obstacle[] REEF_LARGE = {
        new Obstacle.TeardropObstacle(new Translation2d(4.49, 4), 1.4, 2.5, 1.0, 0.7, 1.2),
        new Obstacle.TeardropObstacle(new Translation2d(13.08, 4), 1.4, 2.5, 1.0, 0.7, 1.2),
    };

    private static final Obstacle[] WALL =
            new Obstacle[] {
                new HorizontalObstacle(0.0, 0.3, .75, true),
                new HorizontalObstacle(FIELD_WIDTH, 0.3, .75, false),
                new VerticalObstacle(0.0, 0.3, .75, true),
                new VerticalObstacle(FIELD_LENGTH, 0.3, .75, false),
            };
    private static final Obstacle[] BARGE = {
        new VerticalObstacle(7.55, 0.3, 0.3, false), new VerticalObstacle(10, 0.3, 0.3, true)
    };

    private static final Rectangle2d FIELD =
            new Rectangle2d(
                    FieldConstants2025.POSE2D_CENTER,
                    FieldConstants2025.FIELD_LENGTH,
                    FieldConstants2025.FIELD_WIDTH);

    public static final Obstacle[] ALL_OBSTACLES;

    static {
        ALL_OBSTACLES = Stream.of(WALL, REEF_LARGE).flatMap(Stream::of).toArray(Obstacle[]::new);
    }

    public enum PathObstacles {
        CLOSE_LEFT_REEF(
                FieldConstants2025.Reef.Side.CLOSE_LEFT.face,
                WALL,
                BARGE,
                REEF_VERTICES,
                REEF_SMALL,
                reefSides(0)),
        CLOSE_MID_REEF(
                FieldConstants2025.Reef.Side.CLOSE_MID.face,
                WALL,
                BARGE,
                REEF_VERTICES,
                REEF_SMALL,
                reefSides(1)),
        CLOSE_RIGHT_REEF(
                FieldConstants2025.Reef.Side.CLOSE_RIGHT.face,
                WALL,
                BARGE,
                REEF_VERTICES,
                REEF_SMALL,
                reefSides(2)),
        FAR_LEFT_REEF(
                FieldConstants2025.Reef.Side.FAR_LEFT.face,
                WALL,
                BARGE,
                REEF_VERTICES,
                REEF_SMALL,
                reefSides(3)),
        FAR_MID_REEF(
                FieldConstants2025.Reef.Side.FAR_MID.face,
                WALL,
                BARGE,
                REEF_VERTICES,
                REEF_SMALL,
                reefSides(4)),
        FAR_RIGHT_REEF(
                FieldConstants2025.Reef.Side.FAR_RIGHT.face,
                WALL,
                BARGE,
                REEF_VERTICES,
                REEF_SMALL,
                reefSides(5)),
        CAGE(WALL, REEF_LARGE),
        Other(WALL, REEF_LARGE);

        public final Rectangle2d blueHitBox;
        public final Rectangle2d redHitBox;
        public final Obstacle[] obstacles;

        private static Rectangle2d faceHitBox(Pose2d face) {
            final Transform2d transform =
                    new Transform2d(new Translation2d(0.5, 0.0), Rotation2d.kZero);
            return new Rectangle2d(face.plus(transform), 1.5, 1.0);
        }

        PathObstacles(Pose2d hitBox, Obstacle[]... obstacles) {
            this(faceHitBox(hitBox), obstacles);
        }

        PathObstacles(Obstacle[]... obstacles) {
            this(FIELD, obstacles);
        }

        PathObstacles(Rectangle2d hitBox, Obstacle[]... obstacles) {
            this.blueHitBox = hitBox;
            this.redHitBox = faceHitBox(AllianceSymmetry.flip(hitBox.getCenter()));
            int totalLength = 0;
            for (var obstacleSet : obstacles) {
                totalLength += obstacleSet.length;
            }
            Obstacle[] all = new Obstacle[totalLength];
            int i = 0;
            for (var obstacleSet : obstacles) {
                for (var obstacle : obstacleSet) {
                    all[i] = obstacle;
                    i++;
                }
            }

            this.obstacles = all;
        }

        public static PathObstacles fromReefSide(FieldConstants2025.Reef.Side side) {
            return switch (side) {
                case CLOSE_LEFT -> CLOSE_LEFT_REEF;
                case CLOSE_MID -> CLOSE_MID_REEF;
                case CLOSE_RIGHT -> CLOSE_RIGHT_REEF;
                case FAR_LEFT -> FAR_LEFT_REEF;
                case FAR_MID -> FAR_MID_REEF;
                case FAR_RIGHT -> FAR_RIGHT_REEF;
            };
        }

        public boolean insideHitBox(Translation2d position) {
            return (AllianceSymmetry.isBlue() && blueHitBox.contains(position))
                    || (AllianceSymmetry.isRed() && redHitBox.contains(position));
        }
    }

    /**
     * 2025 Reefscape field constants, ported from 2025_Reefscape. All units are in meters and poses
     * have a blue alliance origin.
     */
    public static class FieldConstants2025 {
        public static final double FIELD_LENGTH = 690.876 * Conv.INCHES_TO_METERS;
        public static final double FIELD_WIDTH = 317 * Conv.INCHES_TO_METERS;
        public static final double STARTING_LINE_X = inchesToMeters(298.438);

        public static class Processor {
            public static final Pose2d CENTER_FACE =
                    new Pose2d(inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
        }

        public static class Barge {
            public static final Translation2d FAR_CAGE =
                    new Translation2d(inchesToMeters(345.428), inchesToMeters(286.779));
            public static final Translation2d MIDDLE_CAGE =
                    new Translation2d(inchesToMeters(345.428), inchesToMeters(242.855));
            public static final Translation2d CLOSE_CAGE =
                    new Translation2d(inchesToMeters(345.428), inchesToMeters(199.947));

            // Measured from floor to bottom of cage
            public static final double DEEP_CAGE_HEIGHT = inchesToMeters(3.125);
            public static final double SHALLOW_CAGE_HEIGHT = inchesToMeters(30.125);
        }

        public static class CoralStation {
            public static final Pose2d LEFT_CENTER_FACE =
                    new Pose2d(
                            inchesToMeters(33.526),
                            inchesToMeters(291.176),
                            Rotation2d.fromDegrees(90 - 144.011));
            public static final Pose2d RIGHT_CENTER_FACE =
                    new Pose2d(
                            inchesToMeters(33.526),
                            inchesToMeters(25.824),
                            Rotation2d.fromDegrees(144.011 - 90));
        }

        public static class AutoGamePieces {
            // Measured from the center of the ice cream
            public static final Pose2d LEFT_GAMEPIECE_STACK =
                    new Pose2d(inchesToMeters(48), inchesToMeters(230.5), new Rotation2d());
            public static final Pose2d MIDDLE_GAMEPIECE_STACK =
                    new Pose2d(inchesToMeters(48), inchesToMeters(158.5), new Rotation2d());
            public static final Pose2d RIGHT_GAMEPIECE_STACK =
                    new Pose2d(inchesToMeters(48), inchesToMeters(86.5), new Rotation2d());
        }

        public static final class Reef {
            public static final Translation2d CENTER =
                    new Translation2d(inchesToMeters(176.746), inchesToMeters(158.501));
            private static final Pose2d CENTER_POSE = new Pose2d(CENTER, Rotation2d.kZero);
            private static final Translation2d FACE_OFFSET =
                    new Translation2d(inchesToMeters(32.75), 0.0);

            public enum BranchHeight implements StructSerializable {
                L4(inchesToMeters(72), -90 * Conv.DEGREES_TO_RADIANS),
                L3(inchesToMeters(47.625), -35 * Conv.DEGREES_TO_RADIANS),
                L2(inchesToMeters(31.875), -35 * Conv.DEGREES_TO_RADIANS),
                L1(inchesToMeters(18), 0);

                BranchHeight(double height, double pitch) {
                    this.height = height;
                    this.pitch = pitch;
                }

                public final double height;
                public final double pitch;

                public static final Struct<BranchHeight> struct =
                        ProceduralStructGenerator.genEnum(BranchHeight.class);
            }

            public enum Side implements StructSerializable {
                CLOSE_LEFT(Rotation2d.fromDegrees(120.0)),
                CLOSE_MID(Rotation2d.fromDegrees(180.0)),
                CLOSE_RIGHT(Rotation2d.fromDegrees(-120.0)),
                FAR_LEFT(Rotation2d.fromDegrees(60.0)),
                FAR_MID(Rotation2d.fromDegrees(0.0)),
                FAR_RIGHT(Rotation2d.fromDegrees(-60.0));

                /**
                 * The position of the center of the face of the reef pointing away from the center
                 * of the reef.
                 */
                public final Pose2d face;

                private Side(Rotation2d angle) {
                    this.face =
                            Reef.CENTER_POSE.plus(
                                    new Transform2d(FACE_OFFSET.rotateBy(angle), angle));
                }

                public static final Pose2d[] FACES = {
                    CLOSE_LEFT.face,
                    CLOSE_MID.face,
                    CLOSE_RIGHT.face,
                    FAR_LEFT.face,
                    FAR_MID.face,
                    FAR_RIGHT.face
                };

                private static final double BRANCH_OFFSET = inchesToMeters(6.5);

                private Pose2d scorePose(double distFromFace, double yOffset) {
                    Translation2d t =
                            FAR_MID.face
                                    .getTranslation()
                                    .plus(new Translation2d(distFromFace, yOffset))
                                    .rotateAround(CENTER, this.face.getRotation());
                    return new Pose2d(t, this.face.getRotation().rotateBy(Rotation2d.kPi));
                }

                public Pose2d alignScoreLeft(double distFromFace, double yOffset) {
                    return scorePose(distFromFace, -BRANCH_OFFSET + yOffset);
                }

                public Pose2d alignScoreRight(double distFromFace, double yOffset) {
                    return scorePose(distFromFace, BRANCH_OFFSET + yOffset);
                }

                public Pose2d alignScoreCenter(double distFromFace, double yOffset) {
                    return scorePose(distFromFace, yOffset);
                }

                public static final Struct<Side> struct =
                        ProceduralStructGenerator.genEnum(Side.class);
            }
        }

        public enum FaceSubLocation implements StructSerializable {
            LEFT,
            RIGHT,
            CENTER;

            public static final Struct<FaceSubLocation> struct =
                    ProceduralStructGenerator.genEnum(FaceSubLocation.class);
        }

        public static final Translation2d TRANSLATION2D_CENTER =
                new Translation2d(FIELD_LENGTH / 2.0, FIELD_WIDTH / 2.0);
        public static final Pose2d POSE2D_CENTER =
                new Pose2d(TRANSLATION2D_CENTER, Rotation2d.kZero);
    }
}
