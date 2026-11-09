package wayfinder.repulsorField;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.Logger;

public abstract class Obstacle {
    double strength;
    boolean positive;

    public Obstacle(double strength, boolean positive) {
        this.strength = strength;
        this.positive = positive;
    }

    public abstract Translation2d getForceAtPosition(Translation2d position, Translation2d goal);

    protected double distToForceMag(double dist, double maxRange) {
        if (MathUtil.isNear(0, dist, 1e-2)) {
            dist = 1e-2;
        }
        var forceMag = strength / (dist * dist);
        forceMag -= strength / (maxRange * maxRange);
        forceMag *= positive ? 1 : -1;
        return forceMag;
    }

    protected double rotateBy(double radians, double rhs) {
        return Math.cos(radians) * rhs;
    }

    public static class SnowmanObstacle extends Obstacle {
        final Translation2d loc;
        final double primaryMaxRange;
        final double secondaryDistance;
        final double secondaryMaxRange;
        final double secondaryStrengthRatio;

        public SnowmanObstacle(
                Translation2d loc,
                double primaryStrength,
                double primaryMaxRange,
                double secondaryDistance,
                double secondaryStrength,
                double secondaryMaxRange) {
            super(primaryStrength, true);
            this.loc = loc;
            this.primaryMaxRange = primaryMaxRange;
            this.secondaryDistance = secondaryDistance;
            this.secondaryMaxRange = secondaryMaxRange;
            secondaryStrengthRatio = primaryStrength / secondaryStrength;
        }

        public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
            var targetToLoc = loc.minus(target);
            var targetToLocAngle = targetToLoc.getAngle();
            var sidewaysCircle =
                    new Translation2d(secondaryDistance, targetToLoc.getAngle()).plus(loc);
            var dist = loc.getDistance(position);
            var sidewaysDist = sidewaysCircle.getDistance(position);
            if (dist > primaryMaxRange && sidewaysDist > secondaryMaxRange) {
                return Translation2d.kZero;
            }
            var sidewaysMag =
                    distToForceMag(sidewaysCircle.getDistance(position), primaryMaxRange)
                            / secondaryStrengthRatio;
            var outwardsMag = distToForceMag(loc.getDistance(position), secondaryMaxRange);
            var initial = new Translation2d(outwardsMag, position.minus(loc).getAngle());

            // flip the sidewaysMag based on which side of the goal-sideways circle the robot is on
            var sidewaysTheta =
                    target.minus(position)
                            .getAngle()
                            .minus(position.minus(sidewaysCircle).getAngle());

            double sideways = sidewaysMag * Math.signum(Math.sin(sidewaysTheta.getRadians()));
            var sidewaysAngle = targetToLocAngle.rotateBy(Rotation2d.kCCW_90deg);
            return new Translation2d(sideways, sidewaysAngle).plus(initial);
        }
    }

    /** Represents a Gaussian (bell-curve) obstacle in the field. */
    public static class GaussianObstacle extends Obstacle {
        private final Translation2d loc; // Center peak of the curve (h, k)
        private final double amplitude; // Peak height of the bump (positive or negative)
        private final double
                sigma; // Controls the width/spread of the tall bit (Standard Deviation)
        private final double extension; // Half-width bounds from peak along the base axis
        private final boolean horizontal; // true = peaks left/right, false = peaks up/down
        private final double primaryMaxRange;
        private final String name;

        public GaussianObstacle(
                Translation2d loc,
                double amplitude,
                double sigma,
                double extension,
                double maxRange,
                boolean horizontal,
                double primaryStrength,
                String name) {
            super(primaryStrength, true);
            this.loc = loc;
            this.amplitude = amplitude;
            this.sigma = sigma;
            this.extension = extension;
            this.horizontal = horizontal;
            this.primaryMaxRange = maxRange;
            this.name = name;
        }

        public void visualizePath() {
            Pose2d[] path = new Pose2d[20];
            double scaleFactor = extension / 10.0;
            for (int i = 0; i < 20; i++) {
                double independent = (i - 10) * scaleFactor;
                double gaussianExponent = -Math.pow(independent, 2) / (2 * Math.pow(sigma, 2));

                if (!horizontal) {
                    double x_cord = independent + loc.getX();
                    double y_cord = amplitude * Math.exp(gaussianExponent) + loc.getY();
                    path[i] = new Pose2d(x_cord, y_cord, Rotation2d.kZero);
                } else {
                    double y_cord = independent + loc.getY();
                    double x_cord = amplitude * Math.exp(gaussianExponent) + loc.getX();
                    path[i] = new Pose2d(x_cord, y_cord, Rotation2d.kZero);
                }
            }

            Logger.recordOutput("ROBOT/WAYFINDER/CURVE/" + name, path);
        }

        @Override
        public Translation2d getForceAtPosition(Translation2d position, Translation2d goal) {
            double closestX;
            double closestY;
            double normalAngleRad;
            boolean isInsidePocket = false;

            if (!horizontal) {
                // Vertical Gaussian: y = amplitude * exp(-(x-h)^2 / (2*sigma^2)) + k
                closestX =
                        MathUtil.clamp(
                                position.getX(), loc.getX() - extension, loc.getX() + extension);

                double dx = closestX - loc.getX();
                double gaussianExponent = -Math.pow(dx, 2) / (2 * Math.pow(sigma, 2));
                closestY = amplitude * Math.exp(gaussianExponent) + loc.getY();

                // Strict boundary check: within extension and captured under/over the bell curve
                boolean withinExtension = Math.abs(position.getX() - loc.getX()) <= extension;
                if (withinExtension) {
                    double rimY =
                            amplitude * Math.exp(-Math.pow(extension, 2) / (2 * Math.pow(sigma, 2)))
                                    + loc.getY();
                    if (amplitude > 0) {
                        // Upward bump: Inside means caught between the tall peak and the lower rim
                        isInsidePocket = position.getY() <= closestY && position.getY() >= rimY;
                    } else {
                        // Downward bump: Inside means caught between the low peak and the higher
                        // rim
                        isInsidePocket = position.getY() >= closestY && position.getY() <= rimY;
                    }
                }

                // Derivative dy/dx = -(amplitude * (x - h) / sigma^2) * exp(...)
                double tangentSlope =
                        -(amplitude * dx / Math.pow(sigma, 2)) * Math.exp(gaussianExponent);
                normalAngleRad = Math.atan2(1.0, -tangentSlope);
            } else {
                // Horizontal Gaussian: x = amplitude * exp(-(y-k)^2 / (2*sigma^2)) + h
                closestY =
                        MathUtil.clamp(
                                position.getY(), loc.getY() - extension, loc.getY() + extension);

                double dy = closestY - loc.getY();
                double gaussianExponent = -Math.pow(dy, 2) / (2 * Math.pow(sigma, 2));
                closestX = amplitude * Math.exp(gaussianExponent) + loc.getX();

                // Strict boundary check
                boolean withinExtension = Math.abs(position.getY() - loc.getY()) <= extension;
                if (withinExtension) {
                    double rimX =
                            amplitude * Math.exp(-Math.pow(extension, 2) / (2 * Math.pow(sigma, 2)))
                                    + loc.getX();
                    if (amplitude > 0) {
                        // Rightward bump
                        isInsidePocket = position.getX() <= closestX && position.getX() >= rimX;
                    } else {
                        // Leftward bump
                        isInsidePocket = position.getX() >= closestX && position.getX() <= rimX;
                    }
                }

                // Derivative dx/dy = -(amplitude * (y - k) / sigma^2) * exp(...)
                double tangentSlopeInverse =
                        -(amplitude * dy / Math.pow(sigma, 2)) * Math.exp(gaussianExponent);
                normalAngleRad = Math.atan2(-tangentSlopeInverse, 1.0);
            }

            visualizePath();

            Translation2d closestPointOnCurve = new Translation2d(closestX, closestY);
            double distance = position.getDistance(closestPointOnCurve);

            // If outside the bounded pocket AND too far away, drop calculation
            if (!isInsidePocket && (distance > primaryMaxRange || distance < 1e-4)) {
                return new Translation2d(0, 0);
            }

            Rotation2d forceDirection = new Rotation2d(normalAngleRad);

            // Dynamic flip to ensure vector always pushes AWAY from the curve surface
            Translation2d vectorToRobot = position.minus(closestPointOnCurve);
            double dotProduct =
                    vectorToRobot.getX() * forceDirection.getCos()
                            + vectorToRobot.getY() * forceDirection.getSin();

            if (dotProduct < 0) {
                forceDirection = forceDirection.rotateBy(Rotation2d.fromDegrees(180));
            }

            if (isInsidePocket) {
                forceDirection = forceDirection.rotateBy(Rotation2d.fromDegrees(180));
            }

            double distanceToMin;
            if (horizontal) {
                distanceToMin = MathUtil.clamp(Math.abs(position.getY() - loc.getY()), 0, .1);
            } else {
                distanceToMin = MathUtil.clamp(Math.abs(position.getX() - loc.getX()), 0, .1);
            }

            // EJECTION RULE: Apply localized force profiles based on pocket state
            double forceMag =
                    isInsidePocket
                            ? distToForceMag(distanceToMin, extension)
                            : distToForceMag(distance, primaryMaxRange);

            return new Translation2d(forceMag, forceDirection);
        }
    }

    /** Represents a parabolic obstacle in the field. */
    public static class ParabolicObstacle extends Obstacle {
        private final Translation2d loc; // Vertex (h, k)
        private final double a; // Convexity coefficient
        private final double extension; // Half-width bounds from vertex
        private final boolean horizontal; // true = opens left/right, false = opens up/down
        private final double primaryMaxRange;
        private final String name;

        public ParabolicObstacle(
                Translation2d loc,
                double a,
                double extension,
                double maxRange,
                boolean horizontal,
                double primaryStrength,
                String name) {
            super(primaryStrength, true);
            this.loc = loc;
            this.a = a;
            this.extension = extension;
            this.horizontal = horizontal;
            this.primaryMaxRange = maxRange;
            this.name = name;
        }

        public void visualizePath() {
            Pose2d[] path = new Pose2d[20];
            double scaleFactor = extension / 10.0; // Adjust scale factor as needed
            for (int i = 0; i < 20; i++) {
                // Calculate path points (simplified for demonstration)
                double y_cord = (i - 10) * scaleFactor + loc.getY();
                double x_cord = a * Math.pow(y_cord - loc.getY(), 2) + loc.getX();
                path[i] = new Pose2d(x_cord, y_cord, Rotation2d.kZero);
            }

            Logger.recordOutput("ROBOT/WAYFINDER/CURVE/" + name, path);
        }

        @Override
        public Translation2d getForceAtPosition(Translation2d position, Translation2d goal) {
            double closestX;
            double closestY;
            double normalAngleRad;
            // System.out.println("Processing parabolic obstacle: " + name);
            boolean isInsidePocket = false;

            if (!horizontal) {
                // Vertical parabola: y = a*(x - h)^2 + k
                closestX =
                        MathUtil.clamp(
                                position.getX(), loc.getX() - extension, loc.getX() + extension);
                closestY = a * Math.pow(closestX - loc.getX(), 2) + loc.getY();

                // Strict boundary check: Must be within the X extensions AND between the curve and
                // the rim
                boolean withinExtension = Math.abs(position.getX() - loc.getX()) <= extension;
                if (withinExtension) {
                    double rimY = a * Math.pow(extension, 2) + loc.getY();
                    if (a > 0) {
                        isInsidePocket = position.getY() >= closestY && position.getY() <= rimY;
                    } else {
                        isInsidePocket = position.getY() <= closestY && position.getY() >= rimY;
                    }
                }

                double tangentSlope = 2 * a * (closestX - loc.getX());
                normalAngleRad = Math.atan2(1.0, -tangentSlope);
            } else {
                // Horizontal parabola: x = a*(y - k)^2 + h
                closestY =
                        MathUtil.clamp(
                                position.getY(), loc.getY() - extension, loc.getY() + extension);
                closestX = a * Math.pow(closestY - loc.getY(), 2) + loc.getX();

                // Strict boundary check: Must be within the Y extensions AND between the curve and
                // the rim
                boolean withinExtension = Math.abs(position.getY() - loc.getY()) <= extension;
                if (withinExtension) {
                    double rimX = a * Math.pow(extension, 2) + loc.getX();
                    if (a > 0) {
                        isInsidePocket = position.getX() >= closestX && position.getX() <= rimX;
                    } else {
                        isInsidePocket = position.getX() <= closestX && position.getX() >= rimX;
                    }
                }

                double tangentSlopeInverse = 2 * a * (closestY - loc.getY());

                normalAngleRad = Math.atan2(-tangentSlopeInverse, 1.0);
            }

            visualizePath();

            Translation2d closestPointOnCurve = new Translation2d(closestX, closestY);
            double distance = position.getDistance(closestPointOnCurve);

            // If outside the bounded pocket AND too far away, drop calculation
            if (!isInsidePocket && (distance > primaryMaxRange || distance < 1e-4)) {
                return new Translation2d(0, 0);
            }

            Rotation2d forceDirection = new Rotation2d(normalAngleRad);

            // Dynamic flip to ensure vector always pushes AWAY from the curve
            Translation2d vectorToRobot = position.minus(closestPointOnCurve);
            double dotProduct =
                    vectorToRobot.getX() * forceDirection.getCos()
                            + vectorToRobot.getY() * forceDirection.getSin();

            if (dotProduct < 0) {
                forceDirection = forceDirection.rotateBy(Rotation2d.fromDegrees(180));
            }

            if (isInsidePocket) {
                forceDirection = forceDirection.rotateBy(Rotation2d.fromDegrees(180));
            }

            double distanceToMin;
            if (horizontal) {
                distanceToMin = MathUtil.clamp(Math.abs(position.getY() - loc.getY()), 0, .4);
            } else {
                distanceToMin = MathUtil.clamp(Math.abs(position.getX() - loc.getX()), 0, .4);
            }

            // EJECTION RULE: If trapped in the shaded region, apply max force
            double forceMag =
                    isInsidePocket
                            ? distToForceMag(distanceToMin, extension)
                            : distToForceMag(distance, primaryMaxRange);

            return new Translation2d(forceMag, forceDirection);
        }
    }

    public static class RectangleObstacle extends Obstacle {

        private final Translation2d locBottomLeft;
        private final Translation2d dimensions; // width and height
        private final double primaryMaxRange;

        public RectangleObstacle(
                Translation2d locBottomLeft,
                Translation2d dimensions,
                double primaryStrength,
                double primaryMaxRange) {
            super(primaryStrength, true);
            this.locBottomLeft = locBottomLeft;
            this.dimensions = dimensions;
            this.primaryMaxRange = primaryMaxRange;
        }

        @Override
        public Translation2d getForceAtPosition(Translation2d position, Translation2d goal) {
            // 1. Calculate the boundaries of the rectangle
            double minX = locBottomLeft.getX();
            double maxX = minX + dimensions.getX();
            double minY = locBottomLeft.getY();
            double maxY = minY + dimensions.getY();

            // 2. Clamp the robot's position to the bounds of the rectangle
            // to find the closest point on (or inside) the obstacle.
            double closestX = MathUtil.clamp(position.getX(), minX, maxX);
            double closestY = MathUtil.clamp(position.getY(), minY, maxY);
            Translation2d closestPoint = new Translation2d(closestX, closestY);

            // 3. Calculate distance and vector from the closest point to the robot
            Translation2d surfaceToRobot = position.minus(closestPoint);
            double dist = surfaceToRobot.getNorm();

            // If the robot is perfectly on the edge or inside, give it a tiny offset
            // so we don't divide by zero or lose the direction vector.
            if (MathUtil.isNear(0, dist, 1e-2)) {
                dist = 1e-2;
                // Default push direction upwards if we are exactly dead-center inside
                surfaceToRobot = new Translation2d(0, dist);
            }

            // 4. If the robot is outside the maximum field of influence, no force is applied
            if (dist > primaryMaxRange) {
                return Translation2d.kZero;
            }

            // 5. Calculate the force magnitude using the base class function
            double forceMag = distToForceMag(dist, primaryMaxRange);
            if (forceMag >= 1000) {
                // System.out.println("Invalid force detected!");
            }
            // 6. Return the force vector matching the direction away from the rectangle
            return new Translation2d(forceMag, surfaceToRobot.getAngle());
        }
    }

    public static class TeardropObstacle extends Obstacle {
        private final Translation2d loc;
        private final double primaryMaxRange;
        private final double primaryRadius;
        private final double tailStrength;
        private final double tailDistance;

        // private final MutTranslation2d sidewaysPoint = new MutTranslation2d();
        // private final MutTranslation2d outwardsForce = new MutTranslation2d();
        // private final MutTranslation2d sidewaysForce = new MutTranslation2d();

        public TeardropObstacle(
                Translation2d loc,
                double primaryStrength,
                double primaryMaxRange,
                double primaryRadius,
                double tailStrength,
                double tailLength) {
            super(primaryStrength, true);
            this.loc = loc;
            this.primaryMaxRange = primaryMaxRange;
            this.primaryRadius = primaryRadius;
            this.tailStrength = tailStrength;
            this.tailDistance = tailLength + primaryMaxRange;
        }

        // public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
        //   final Rotation2d targetToLocAngle = loc.minus(target).getAngle();
        //   sidewaysPoint.setPolar(tailDistance, targetToLocAngle);
        //   sidewaysPoint.plusMut(loc);
        //   outwardsForce.setZero();
        //   sidewaysForce.setZero();

        //   final Translation2d positionToLocation = position.minus(loc);
        //   final double positionToLocationDistance = positionToLocation.getNorm();

        //   if (positionToLocationDistance <= primaryMaxRange) {
        //     final double magnitude =
        //         distToForceMag(
        //             Math.max(positionToLocationDistance - primaryRadius, 0),
        //             primaryMaxRange - primaryRadius);
        //     outwardsForce.setPolar(magnitude, positionToLocation.getAngle());
        //   }

        //   final Translation2d positionToLine =
        //       position.minus(loc).rotateBy(targetToLocAngle.unaryMinus());
        //   final double distanceAlongLine = positionToLine.getX();

        //   final double distanceScalar = distanceAlongLine / tailDistance;
        //   if (distanceScalar >= 0 && distanceScalar <= 1) {
        //     final double secondaryMaxRange =
        //         MathUtil.interpolate(primaryMaxRange, 0, distanceScalar * distanceScalar);
        //     final double distanceToLine = Math.abs(positionToLine.getY());
        //     if (distanceToLine > secondaryMaxRange) {
        //       return outwardsForce;
        //     }
        //     final double sidewaysMag =
        //         tailStrength
        //             * (1 - distanceScalar * distanceScalar)
        //             * (secondaryMaxRange - distanceToLine);
        //     // flip the sidewaysMag based on which side of the goal-sideways circle the robot is
        // on
        //     final Rotation2d targetToPositionAngle =
        //         new Rotation2d(target.getX() - position.getX(), target.getY() - position.getY());
        //     final Rotation2d positionToSidewaysPointUnaryAngle =
        //         new Rotation2d(
        //             position.getX() - sidewaysPoint.getX(), -(position.getY() -
        // sidewaysPoint.getY()));
        //     final double sidewaysThetaSin =
        //         (targetToPositionAngle.getCos() * positionToSidewaysPointUnaryAngle.getSin())
        //             + (targetToPositionAngle.getSin() *
        // positionToSidewaysPointUnaryAngle.getCos());
        //     sidewaysForce.setPolar(sidewaysMag * Math.signum(sidewaysThetaSin),
        // targetToLocAngle);
        //     sidewaysForce.rotateByMut(Rotation2d.kCCW_90deg);
        //   }

        //   outwardsForce.plusMut(sidewaysForce);
        //   return outwardsForce;
        // }

        public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
            var targetToLoc = loc.minus(target);
            var targetToLocAngle = targetToLoc.getAngle();
            var sidewaysPoint = new Translation2d(tailDistance, targetToLoc.getAngle()).plus(loc);

            var positionToLocation = position.minus(loc);
            var positionToLocationDistance = positionToLocation.getNorm();
            Translation2d outwardsForce;
            if (positionToLocationDistance <= primaryMaxRange) {
                outwardsForce =
                        new Translation2d(
                                distToForceMag(
                                        Math.max(positionToLocationDistance - primaryRadius, 0),
                                        primaryMaxRange - primaryRadius),
                                positionToLocation.getAngle());
            } else {
                outwardsForce = Translation2d.kZero;
            }

            var positionToLine = position.minus(loc).rotateBy(targetToLocAngle.unaryMinus());
            var distanceAlongLine = positionToLine.getX();

            Translation2d sidewaysForce;
            var distanceScalar = distanceAlongLine / tailDistance;
            if (distanceScalar >= 0 && distanceScalar <= 1) {
                var secondaryMaxRange =
                        MathUtil.interpolate(primaryMaxRange, 0, distanceScalar * distanceScalar);
                var distanceToLine = Math.abs(positionToLine.getY());
                if (distanceToLine <= secondaryMaxRange) {
                    var sidewaysMag =
                            tailStrength
                                    * (1 - distanceScalar * distanceScalar)
                                    * (secondaryMaxRange - distanceToLine);
                    // flip the sidewaysMag based on which side of the goal-sideways circle the
                    // robot is on
                    var sidewaysTheta =
                            target.minus(position)
                                    .getAngle()
                                    .minus(position.minus(sidewaysPoint).getAngle());
                    sidewaysForce =
                            new Translation2d(
                                    sidewaysMag * Math.signum(Math.sin(sidewaysTheta.getRadians())),
                                    targetToLocAngle.rotateBy(Rotation2d.kCCW_90deg));
                } else {
                    sidewaysForce = Translation2d.kZero;
                }
            } else {
                sidewaysForce = Translation2d.kZero;
            }

            return outwardsForce.plus(sidewaysForce);
        }
    }

    public static class HorizontalObstacle extends Obstacle {
        final double y;
        final double maxRange;

        public HorizontalObstacle(double y, double strength, double maxRange, boolean positive) {
            super(strength, positive);
            this.y = y;
            this.maxRange = maxRange;
        }

        @Override
        public Translation2d getForceAtPosition(Translation2d position, Translation2d goal) {
            double dist = Math.abs(position.getY() - y);
            // System.out.println(
            //    "HorizontalObstacle force at position: "
            //            + position.getX()
            //            + ", "
            //            + position.getY());
            if (dist < maxRange) {
                double forceMag = distToForceMag(y - position.getY(), maxRange);
                if (forceMag >= 1000) {
                    // System.out.println("Invalid force detected!");
                }
                return new Translation2d(0.0, forceMag);
            }
            return new Translation2d(); // Defaults to (0.0, 0.0)
        }
    }

    public static class VerticalObstacle extends Obstacle {
        final double x;
        final double maxRange;

        public VerticalObstacle(double x, double strength, double maxRange, boolean positive) {
            super(strength, positive);
            this.x = x;
            this.maxRange = maxRange;
        }

        @Override
        public Translation2d getForceAtPosition(Translation2d position, Translation2d goal) {
            double dist = Math.abs(position.getX() - x);
            // System.out.println(
            //      "VerticalObstacle force at position: "
            //              + position.getX()
            //              + ", "
            //              + position.getY());
            if (dist < maxRange) {
                double forceMag = distToForceMag(x - position.getX(), maxRange);
                if (forceMag >= 1000) {
                    // System.out.println("Invalid force detected!");
                }
                return new Translation2d(forceMag, 0.0);
            }
            return new Translation2d(); // Defaults to (0.0, 0.0)
        }
    }
}
