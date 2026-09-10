package igknighters.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.Robot;
import igknighters.constants.Conv;
import igknighters.constants.FieldConstants;
import igknighters.subsystems.swerve.Swerve;
import igknighters.util.log.Log;
import java.util.ArrayList;

// rename
public class Repulsor {
    public enum obstacleType {
        CIRCLE,
        SQUARE,
        SAFE_ZONE;
    }

    static boolean beenPublished = false;

    /**
     * {@summary}Holds the data for an obstacle used in the repulsor field navigation system.
     *
     * @param obstaclePose The Pose2d representing the position of the obstacle rotation is ignored
     *     measured from center of obstacle.
     * @param strength The repulsion strength of the obstacle. 1 is a good starting point.
     * @param width The width of the obstacle (center to edge)(used for visualization or collision
     *     detection).
     * @param height The height of the obstacle (center to edge)(used for visualization or collision
     *     detection).
     */
    public record obstacle(
            Pose2d obstaclePose, double strength, double width, double height, obstacleType type) {}

    static double maxTime = 0.0;

    static RepulsorVisualizer visualizer = new RepulsorVisualizer();

    // finds X repulsive force by summing all the forces of the obstacles
    public static double getXRepulse(Pose2d currentPose, ArrayList<Repulsor.obstacle> obstacles) {
        double currentTime = RobotController.getFPGATime() * 1000.0; // microseconds to milliseconds

        if (!Robot.consts.disableAllLogs()) {
            Log.log("ROBOT/Commands/repulsor/Time", currentTime);
        }
        double xRepelForce = 0.0;
        for (Repulsor.obstacle obs : obstacles) {
            if (obs.type == obstacleType.CIRCLE) {
                double dist =
                        Math.hypot(
                                obs.obstaclePose.getX() - currentPose.getX(),
                                obs.obstaclePose.getY() - currentPose.getY());
                // determine if obstacle X is above or below robot position to add or subtract force
                if (obs.obstaclePose.getX() - currentPose.getX() > 0) {
                    // uses the function R=(e^OBSTACLE_STRENGTH*e^(2-TANGENT_DISTANCE)*X_DISTANCE)/3
                    xRepelForce +=
                            (Math.pow(Math.E, obs.strength) * Math.pow(Math.E, 2 - dist))
                                    * (obs.obstaclePose.getX() - currentPose.getX())
                                    / 3;
                } else {
                    xRepelForce -=
                            (Math.pow(Math.E, obs.strength) * Math.pow(Math.E, 2 - dist))
                                    * (currentPose.getX() - obs.obstaclePose.getX())
                                    / 3;
                }
            }
            // if robot is too close to walls set repel to 0
            if (currentPose.getX() < 0 + 30 * Conv.INCHES_TO_METERS
                    || currentPose.getX() > FieldConstants.X_FIELD - 30 * Conv.INCHES_TO_METERS) {
                xRepelForce = 0;
            }
        }
        double deltaTime = Timer.getFPGATimestamp() * 1000 - currentTime;

        if (!Robot.consts.disableAllLogs()) {
            Log.log("ROBOT/Commands/repulsor/DeltaTime", deltaTime);
        }
        if (deltaTime > maxTime) {
            maxTime = deltaTime;

            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/Commands/repulsor/MaxDeltaTime", maxTime);
            }
        }
        return -xRepelForce;
    }

    public static double getXGoal(Pose2d currentPose, Pose2d target) {
        double xGoalDist = target.getX() - currentPose.getX();
        if (!Robot.consts.disableAllLogs()) {
            Log.log("ROBOT/Commands/repulsor/xGoalDist", target.getX() - currentPose.getX());
        }
        return xGoalDist;
    }

    // finds X repulsive force by summing all the forces of the obstacles
    public static double getYRepulse(
            Pose2d currentPose, ArrayList<Repulsor.obstacle> obstacles, Pose2d target) {
        double yRepelForce = 0.0;
        double currentTime = RobotController.getFPGATime() * 1000.0; // microseconds to milliseconds
        if (!Robot.consts.disableAllLogs()) {
            Log.log("ROBOT/Commands/repulsor/Time", currentTime);
        }
        for (Repulsor.obstacle obs : obstacles) {
            if (obs.type == obstacleType.CIRCLE) {
                double dist =
                        Math.hypot(
                                obs.obstaclePose.getX() - currentPose.getX(),
                                obs.obstaclePose.getY() - currentPose.getY());
                // determine if obstacle X is above or below robot position to add or subtract force
                if (obs.obstaclePose.getY() - currentPose.getY() > 0) {
                    yRepelForce +=
                            (Math.pow(Math.E, obs.strength) * Math.pow(Math.E, 2 - dist))
                                    * (obs.obstaclePose.getY() - currentPose.getY())
                                    / 3;
                    // }
                } else {
                    yRepelForce -=
                            (Math.pow(Math.E, obs.strength) * Math.pow(Math.E, 2 - dist))
                                    * (currentPose.getY() - obs.obstaclePose.getY())
                                    / 3;
                }
            }
            // if robot is in a safe zone, do not affect y repulsion
            for (Repulsor.obstacle safezones : obstacles) {
                if (safezones.type == obstacleType.SAFE_ZONE
                        && currentPose.getX() >= obs.obstaclePose.getX() - obs.width
                        && currentPose.getX() <= obs.obstaclePose.getX() + obs.width
                        && currentPose.getY() >= obs.obstaclePose.getY() - obs.height
                        && currentPose.getY() <= obs.obstaclePose.getY() + obs.height) {
                    yRepelForce = getYGoal(currentPose, target);
                }
            }
            // if directly aligned in the y with a obstacle, it will push up/down to avoid getting
            // stuck
            if (obs.type != obstacleType.SAFE_ZONE
                    && Math.abs(currentPose.getY() - obs.obstaclePose.getY()) < .3) {
                yRepelForce = 0;
                if ((currentPose.getX() < 182.11 * Conv.INCHES_TO_METERS
                                || currentPose.getX()
                                        > FieldConstants.X_FIELD - 182.11 * Conv.INCHES_TO_METERS)
                        && Math.abs(currentPose.getX() - obs.obstaclePose.getX()) > 1) {
                    if (currentPose.getY() < FieldConstants.Y_FIELD / 2) {
                        yRepelForce += Math.abs(currentPose.getX() - obs.obstaclePose.getX()) * 2;
                    } else if (currentPose.getY() > FieldConstants.Y_FIELD / 2) {
                        yRepelForce -= Math.abs(currentPose.getX() - obs.obstaclePose.getX()) * 2;
                    }
                }
            }
        }
        double deltaTime = Timer.getFPGATimestamp() * 1000 - currentTime;
        if (!Robot.consts.disableAllLogs()) {
            Log.log("ROBOT/Commands/repulsor/DeltaTime", deltaTime);
        }
        if (deltaTime > maxTime) {
            maxTime = deltaTime;
            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/Commands/repulsor/MaxDeltaTime", maxTime);
            }
        }
        // if in front of the hubs, there will be a up/down force to get robot to move towards one
        // side
        if (currentPose.getY() < FieldConstants.Y_FIELD / 2 && currentPose.getY() > 50.35) {
            yRepelForce += .5;
        } else if (currentPose.getY() > FieldConstants.Y_FIELD / 2
                && currentPose.getY() < FieldConstants.Y_FIELD - 50.59 * Conv.INCHES_TO_METERS) {
            yRepelForce -= .5;
        }
        return -yRepelForce;
    }

    // overloaded method for y repulsion that doesn't take target, used for teleop where we just
    // want to repel from obstacles and not be attracted to a target
    public static double getYRepulse(Pose2d currentPose, ArrayList<Repulsor.obstacle> obstacles) {
        double yRepelForce = 0.0;
        double currentTime = RobotController.getFPGATime() * 1000.0; // microseconds to milliseconds
        if (!Robot.consts.disableAllLogs()) {
            Log.log("Commands/repulsor/Time", currentTime);
        }
        for (Repulsor.obstacle obs : obstacles) {
            if (obs.type == obstacleType.CIRCLE) {
                double dist =
                        Math.hypot(
                                obs.obstaclePose.getX() - currentPose.getX(),
                                obs.obstaclePose.getY() - currentPose.getY());
                // determine if obstacle X is above or below robot position to add or subtract force
                if (obs.obstaclePose.getY() - currentPose.getY() > 0) {
                    yRepelForce +=
                            (Math.pow(Math.E, obs.strength) * Math.pow(Math.E, 2 - dist))
                                    * (obs.obstaclePose.getY() - currentPose.getY())
                                    / 3;
                    // }
                } else {
                    yRepelForce -=
                            (Math.pow(Math.E, obs.strength) * Math.pow(Math.E, 2 - dist))
                                    * (currentPose.getY() - obs.obstaclePose.getY())
                                    / 3;
                }
            }
            // if directly aligned in the y with a obstacle, it will push up/down to avoid getting
            // stuck
            if (obs.type != obstacleType.SAFE_ZONE
                    && Math.abs(currentPose.getY() - obs.obstaclePose.getY()) < .3) {
                yRepelForce = 0;
                if ((currentPose.getX() < 182.11 * Conv.INCHES_TO_METERS
                                || currentPose.getX()
                                        > FieldConstants.X_FIELD - 182.11 * Conv.INCHES_TO_METERS)
                        && Math.abs(currentPose.getX() - obs.obstaclePose.getX()) > 1) {
                    if (currentPose.getY() < FieldConstants.Y_FIELD / 2) {
                        yRepelForce += Math.abs(currentPose.getX() - obs.obstaclePose.getX()) * 2;
                    } else if (currentPose.getY() > FieldConstants.Y_FIELD / 2) {
                        yRepelForce -= Math.abs(currentPose.getX() - obs.obstaclePose.getX()) * 2;
                    }
                }
            }
        }
        double deltaTime = Timer.getFPGATimestamp() * 1000 - currentTime;
        if (!Robot.consts.disableAllLogs()) {
            Log.log("Commands/repulsor/DeltaTime", deltaTime);
        }
        if (deltaTime > maxTime) {
            maxTime = deltaTime;
            if (!Robot.consts.disableAllLogs()) {
                Log.log("Commands/repulsor/MaxDeltaTime", maxTime);
            }
        }
        // if in front of the hubs, there will be a up/down force to get robot to move towards one
        // side
        if (currentPose.getY() < FieldConstants.Y_FIELD / 2 && currentPose.getY() > 50.35) {
            yRepelForce += .5;
        } else if (currentPose.getY() > FieldConstants.Y_FIELD / 2
                && currentPose.getY() < FieldConstants.Y_FIELD - 50.59 * Conv.INCHES_TO_METERS) {
            yRepelForce -= .5;
        }
        return -yRepelForce;
    }

    public static double getYGoal(Pose2d currentPose, Pose2d target) {
        double yGoalDist = target.getY() - currentPose.getY();
        if (!Robot.consts.disableAllLogs()) {
            Log.log("ROBOT/Commands/repulsor/yGoalDist", target.getY() - currentPose.getY());
        }
        return yGoalDist;
    }

    public static Command moveWithRepulsor(Swerve swerve, Pose2d targetPose) {
        ArrayList<obstacle> obstacles = FieldConstants.OBSTACLES.ALL_OBSTACLES;
        // FieldVisualizer.getInstance().updateDrivingTarget(targetPose);
        final SwerveRequest.FieldCentric m_driveRequest =
                new SwerveRequest.FieldCentric()
                        .withDeadband(
                                Robot.consts
                                                .swerve()
                                                .getCommonSwerveConsts()
                                                .getMaxSpeedMetersPerSecond()
                                        * 0.05)
                        .withRotationalDeadband(
                                RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 0.05)
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo);
        @SuppressWarnings("resource")
        final PIDController thetaController = new PIDController(0.1, 0.0, 0.0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        return swerve.run(
                () -> {
                    Pose2d currentPose = swerve.getState().Pose;
                    double xVelo =
                            10
                                    * -(getXGoal(currentPose, targetPose)
                                            + getXRepulse(currentPose, obstacles));
                    if (!Robot.consts.disableAllLogs()) {
                        Log.log(
                                "ROBOT/Commands/repulsor/xRepel",
                                getXRepulse(currentPose, obstacles));
                    }
                    double yVelo =
                            10
                                    * -(getYGoal(currentPose, targetPose)
                                            + getYRepulse(currentPose, obstacles, targetPose));
                    if (!Robot.consts.disableAllLogs()) {
                        Log.log(
                                "Commands/repulsor/yRepel",
                                getYRepulse(currentPose, obstacles, targetPose));
                    }
                    double omega =
                            thetaController.calculate(
                                    currentPose.getRotation().getRadians(),
                                    targetPose.getRotation().getRadians());
                    RepulsorVisualizer.update(
                            Math.atan2(
                                    getYGoal(currentPose, targetPose),
                                    getXGoal(currentPose, targetPose)),
                            Math.atan2(
                                    getYRepulse(currentPose, obstacles, targetPose),
                                    getXRepulse(currentPose, obstacles)),
                            Math.hypot(
                                    getYGoal(currentPose, targetPose),
                                    getXGoal(currentPose, targetPose)),
                            Math.hypot(
                                    getYRepulse(currentPose, obstacles, targetPose),
                                    getXRepulse(currentPose, obstacles)));

                    omega *= 20.0;
                    if (!Robot.consts.disableAllLogs()) {
                        Log.log("ROBOT/Commands/repulsor/Omega", omega);
                    }
                    swerve.setControl(
                            m_driveRequest
                                    .withVelocityX(xVelo)
                                    .withVelocityY(yVelo)
                                    .withRotationalRate(omega));
                });
    }
}
