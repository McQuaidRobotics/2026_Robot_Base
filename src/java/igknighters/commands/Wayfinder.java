package igknighters.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.Robot;
import igknighters.constants.FieldConstants;
import igknighters.subsystems.swerve.Swerve;
import java.util.Set;
import vroom.Fields.EmptyField;
import vroom.PathFollower;
import vroom.PathPlanner;

public class Wayfinder {
    static PathPlanner pathPlanner = new PathPlanner(new EmptyField(), 4);
    static PathFollower pathFollower = new PathFollower(1.0, 0, 0);

    public static Pose2d getSafeSpotLocation() {
        if (Robot.isBlue()) {
            return new Pose2d(0.5, 1, new Rotation2d());
        } else {
            return new Pose2d(FieldConstants.X_FIELD - .5, 0.5, new Rotation2d());
        }
    }

    public static Command driveToSafeSpot(Swerve swerve) {
        return Commands.defer(
                () -> {
                    return pathFollower.createFollowPathCommand(
                            swerve,
                            pathPlanner.generateTimestampedPath(
                                    swerve.getState().Pose, getSafeSpotLocation()),
                            pathPlanner);
                },
                Set.of(swerve));
    }

    public static Command driveToTarget(Swerve swerve, Pose2d target) {

        return Commands.defer(
                () -> {
                    return pathFollower.createFollowPathCommand(
                            swerve,
                            pathPlanner.generateTimestampedPath(swerve.getState().Pose, target),
                            pathPlanner);
                },
                Set.of(swerve));
    }
}
