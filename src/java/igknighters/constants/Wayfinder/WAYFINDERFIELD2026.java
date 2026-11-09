package igknighters.constants.Wayfinder;

import edu.wpi.first.math.geometry.Translation2d;
import igknighters.constants.FieldConstants;
import java.util.stream.Stream;
import wayfinder.repulsorField.Obstacle;

public class WAYFINDERFIELD2026 {

    private static final Translation2d[] BUMP_VERTICIES =
            new Translation2d[] {
                new Translation2d(
                        FieldConstants.BUMP.BUMP_1_X_METERS
                                - FieldConstants.BUMP.HALF_BUMP_X_METERS,
                        FieldConstants.BUMP.BUMP_1_Y_METERS
                                - FieldConstants.BUMP.HALF_Y_METERS), // Bottom-left vertex BUMP 1
                new Translation2d(
                        FieldConstants.BUMP.BUMP_2_X_METERS
                                - FieldConstants.BUMP.HALF_BUMP_X_METERS,
                        FieldConstants.BUMP.BUMP_2_Y_METERS
                                - FieldConstants.BUMP.HALF_Y_METERS), // Bottom-left vertex BUMP 2
            };

    private static final Obstacle[] BUMP_OBSTACLES_PURE_RECTANGLES;

    static {
        BUMP_OBSTACLES_PURE_RECTANGLES = new Obstacle[BUMP_VERTICIES.length];
        for (int i = 0; i < BUMP_VERTICIES.length; i++) {
            BUMP_OBSTACLES_PURE_RECTANGLES[i] =
                    new Obstacle.RectangleObstacle(
                            BUMP_VERTICIES[i],
                            new Translation2d(
                                    FieldConstants.BUMP.HALF_BUMP_X_METERS * 2,
                                    FieldConstants.BUMP.HALF_Y_METERS * 2),
                            .80,
                            .5);
        }
    }

    private static final Obstacle[] BUMP_FEATURES_STUDS;

    static {
        BUMP_FEATURES_STUDS = new Obstacle[BUMP_VERTICIES.length * 4];

        // Cache full width and height measurements for clarity
        final double width = FieldConstants.BUMP.HALF_BUMP_X_METERS * 2;
        final double height = FieldConstants.BUMP.HALF_Y_METERS * 2;

        // Define the translation offsets for the 4 corners of the bump rectangle
        final Translation2d[] cornerOffsets =
                new Translation2d[] {
                    new Translation2d(0.0, 0.0), // Corner 0: Bottom-Left
                    new Translation2d(width, 0.0), // Corner 1: Bottom-Right
                    new Translation2d(width, height), // Corner 2: Top-Right
                    new Translation2d(0.0, height) // Corner 3: Top-Left
                };

        // Dynamically loop through bumps and offsets
        for (int i = 0; i < BUMP_VERTICIES.length; i++) {
            for (int j = 0; j < 4; j++) {
                // Shift the base vertex by the corresponding corner coordinate vector
                Translation2d studPosition = BUMP_VERTICIES[i].plus(cornerOffsets[j]);

                // Assuming the second parameter expects a radius or identifier, passing j or a
                // constant
                BUMP_FEATURES_STUDS[i * 4 + j] =
                        new Obstacle.SnowmanObstacle(studPosition, .1, .2, .3, .05, .3);
            }
        }
    }

    private static final Obstacle[] BUMP_GAUSSIAN_OBSTACLES;

    static {
        BUMP_GAUSSIAN_OBSTACLES = new Obstacle[BUMP_VERTICIES.length * 4];
        double extrusionLength = 2; // how far it sticks out in meters (Amplitude)
        double maxRange = 4;
        double strength = .5;
        double offsetOffBump = .8;

        double halfX = FieldConstants.BUMP.HALF_BUMP_X_METERS;
        double halfY = FieldConstants.BUMP.HALF_Y_METERS;

        // Tuning parameter for the Gaussian width.
        // Setting sigma to halfY / 2.0 means the extension captures exactly 2 standard
        // deviations (95% of the curve), giving it a smooth, natural bell taper to the corners.
        double sigma = halfY / 2.5;

        for (int i = 0; i < BUMP_VERTICIES.length; i++) {
            // Calculate the physical center-Y line of the bump
            double yCenter = BUMP_VERTICIES[i].getY() + halfY;

            // Base x-coordinates of the flat left and right sides of the bump
            double xBumpLeftBase = BUMP_VERTICIES[i].getX() - offsetOffBump;
            double xBumpRightBase = BUMP_VERTICIES[i].getX() + (halfX * 2) + offsetOffBump;

            // LEFT CAP: Base centered at left edge. Amplitude is negative to bulge left <-
            BUMP_GAUSSIAN_OBSTACLES[i * 4] =
                    new Obstacle.GaussianObstacle(
                            new Translation2d(xBumpLeftBase, yCenter),
                            -extrusionLength, // Negative extends the peak leftward
                            sigma,
                            halfY, // Extension bounds it to the top/bottom corners
                            maxRange,
                            true, // Horizontal profile
                            strength,
                            "BUMP_LEFT OF BUMP: " + (i + 1));

            BUMP_GAUSSIAN_OBSTACLES[i * 4 + 1] =
                    new Obstacle.TeardropObstacle(
                            new Translation2d(xBumpLeftBase - extrusionLength, yCenter),
                            strength / 3.0,
                            .15,
                            .05,
                            strength / 4,
                            -.1);

            // RIGHT CAP: Base centered at right edge. Amplitude is positive to bulge right ->
            BUMP_GAUSSIAN_OBSTACLES[i * 4 + 2] =
                    new Obstacle.GaussianObstacle(
                            new Translation2d(xBumpRightBase, yCenter),
                            extrusionLength, // Positive extends the peak rightward
                            sigma,
                            halfY, // Extension bounds it to top/bottom corners
                            maxRange,
                            true, // Horizontal profile
                            strength,
                            "BUMP_RIGHT OF BUMP: " + (i + 1));

            BUMP_GAUSSIAN_OBSTACLES[i * 4 + 3] =
                    new Obstacle.TeardropObstacle(
                            new Translation2d(xBumpRightBase + extrusionLength, yCenter),
                            strength / 3.0,
                            .15,
                            .05,
                            strength / 4,
                            .1);
        }
    }

    private static final Obstacle[] BUMP_PARABOLA_OBSTACLES;

    static {
        BUMP_PARABOLA_OBSTACLES = new Obstacle[BUMP_VERTICIES.length * 4];
        double extrusionLength = 1.5; // how far it sticks out in meters
        double maxRange = 2.0;
        double strength = .5;

        double halfX = FieldConstants.BUMP.HALF_BUMP_X_METERS;
        double halfY = FieldConstants.BUMP.HALF_Y_METERS;

        // Corrected algebraic derivation for a horizontal parabola: x = a * y^2
        // a = deltaX / (deltaY)^2
        double a = extrusionLength / (halfY * halfY);

        for (int i = 0; i < BUMP_VERTICIES.length; i++) {
            // Calculate the physical center-Y line of the bump
            double yCenter = BUMP_VERTICIES[i].getY() + halfY;

            // WHERE THE VERTEX IS
            double xBumpLeft = BUMP_VERTICIES[i].getX() - extrusionLength;
            double xBumpRight = BUMP_VERTICIES[i].getX() + (halfX * 2) + extrusionLength;

            // LEFT CAP: Vertex is on the left edge. Bends left (negative 'a')
            BUMP_PARABOLA_OBSTACLES[i * 4] =
                    new Obstacle.ParabolicObstacle(
                            new Translation2d(xBumpLeft, yCenter),
                            a, // Negative makes it point left <-
                            halfY, // Extension bounds it to the top/bottom corners
                            maxRange,
                            true, // FIX: Must be horizontal
                            strength,
                            "BUMP_LEFT OF BUMP: " + (i + 1));
            BUMP_PARABOLA_OBSTACLES[i * 4 + 1] =
                    new Obstacle.TeardropObstacle(
                            new Translation2d(xBumpLeft, yCenter),
                            strength / 3.0,
                            .15,
                            .05,
                            strength / 4,
                            -.1);

            // RIGHT CAP: Vertex is on the right edge. Bends right (positive 'a')
            BUMP_PARABOLA_OBSTACLES[i * 4 + 2] =
                    new Obstacle.ParabolicObstacle(
                            new Translation2d(xBumpRight, yCenter),
                            -a, // Positive makes it point right ->
                            halfY, // Extension bounds it to top/bottom corners
                            maxRange,
                            true,
                            strength,
                            "BUMP_RIGHT OF BUMP: " + (i + 1));

            BUMP_PARABOLA_OBSTACLES[i * 4 + 3] =
                    new Obstacle.TeardropObstacle(
                            new Translation2d(xBumpRight, yCenter),
                            strength / 3.0,
                            .15,
                            .05,
                            strength / 4,
                            .1);
        }
    }

    private static final double wallStrength = .3;
    private static final Obstacle[] WALL_OBSTACLES = {
        // the .1 is to help with the fact that the center of the robot is what the pose is. The .1
        // is added to ensure the robot doesn't collide with the wall
        new Obstacle.HorizontalObstacle(0 + .1, wallStrength, .75, true), // bottom wall
        new Obstacle.HorizontalObstacle(
                FieldConstants.Y_FIELD - .1, wallStrength, .75, false), // top wall
        new Obstacle.VerticalObstacle(0 + .1, wallStrength, .75, true), // left wall
        new Obstacle.VerticalObstacle(
                FieldConstants.X_FIELD - .1, wallStrength, .75, false) // right wall
    };

    public static final Obstacle[] ALL_OBSTACLES;

    static {
        ALL_OBSTACLES =
                Stream.of(
                                WALL_OBSTACLES,
                                BUMP_OBSTACLES_PURE_RECTANGLES,
                                BUMP_FEATURES_STUDS,
                                BUMP_PARABOLA_OBSTACLES)
                        .flatMap(Stream::of)
                        .toArray(Obstacle[]::new);
    }
}
