package igknighters.commands.avoidance;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import igknighters.Robot;
import igknighters.commands.SwerveCommands;
import igknighters.subsystems.swerve.Swerve;
import igknighters.util.log.Log;
import java.util.ArrayList;
import java.util.function.Supplier;
import wayfinder.WayfinderManager;
import wayfinder.controllers.Types.ChassisConstraints;
import wayfinder.controllers.Types.Constraints;
import wayfinder.repulsorField.Obstacle;
import wayfinder.repulsorField.Obstacle.TeardropObstacle;
import wayfinder.setpointGenerator.SwerveSetpoint;

/**
 * This is the necesarry logic to allow for the robot to drive while avoiding a obstacle that is not
 * pre determined eg other robot or game pieces
 */
public class DriveWhileAvoiding extends Command {
    private final Swerve swerve;
    private final Supplier<ArrayList<Pose2d>> obstaclePoses;
    private final Supplier<Pose2d> targetSupplier;
    final SwerveRequest.FieldCentric m_driveRequest =
            new SwerveRequest.FieldCentric()
                    .withDeadband(
                            Robot.consts
                                            .swerve()
                                            .getCommonSwerveConsts()
                                            .getMaxSpeedMetersPerSecond()
                                    * .01)
                    .withRotationalDeadband(RotationsPerSecond.of(1.5).in(RadiansPerSecond) * .01)
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                    .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
                    .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

    public DriveWhileAvoiding(
            Swerve swerve,
            Supplier<ArrayList<Pose2d>> obstaclePoses,
            Supplier<Pose2d> targetSupplier) {
        this.swerve = swerve;
        this.obstaclePoses = obstaclePoses;
        this.targetSupplier = targetSupplier;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        Log.log(
                "ROBOT/COMMANDS/SWERVE/AVOIDING",
                "STARTING THE AVOIDANCE COMMAND, Initial target is " + targetSupplier.get());
    }

    @Override
    public void execute() {
        // each object will be treated as a teardrop facing the person to encourage avoidance
        ArrayList<Obstacle> obstacles = new ArrayList<>();
        for (Pose2d obstaclePose : obstaclePoses.get()) {
            // double directionToRobot = Math.atan2(swerve.getState().Pose.getY() -
            // obstaclePose.getY(), swerve.getState().Pose.getX() - obstaclePose.getX());
            obstacles.add(
                    new TeardropObstacle(obstaclePose.getTranslation(), 3.0, 2.0, 1.0, 1.5, .5));
        }

        ArrayList<Double> obstacleIndices =
                WayfinderManager.addObstaclesAndReturnIndex(
                        obstacles.toArray(
                                Obstacle[]::new)); // the robots positions have been added to the
        // field manager and we have the positions to
        // remove after cycle

        SwerveSetpoint setpoint =
                WayfinderManager.calculate(
                        .02,
                        swerve.getState().Pose,
                        swerve.getState().Speeds,
                        targetSupplier.get(),
                        new ChassisConstraints(
                                new Constraints(
                                        Robot.consts
                                                        .swerve()
                                                        .getCommonSwerveConsts()
                                                        .getMaxSpeedMetersPerSecond()
                                                / 2,
                                        4,
                                        8),
                                new Constraints(6, 9, 15)));

        double vx = setpoint.fieldSpeeds().vx();
        double vy = setpoint.fieldSpeeds().vy();
        double omega = setpoint.fieldSpeeds().omega();

        swerve.setControl(
                m_driveRequest
                        .withVelocityX(MetersPerSecond.of(vx))
                        .withVelocityY(MetersPerSecond.of(vy))
                        .withRotationalRate(RadiansPerSecond.of(omega)));

        // we have driven for 20 ms this method will be continuosly looped on so this code will be
        // called continuosly

        // remove the old obstacles because next loop things could have moved and its relatively
        // cheap to add and remove obstacles
        WayfinderManager.removeObstacles(obstacleIndices);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(m_driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
        Log.log(
                "ROBOT/COMMANDS/SWERVE/AVOIDING",
                "ENDING THE AVOIDANCE COMMAND with "
                        + (targetSupplier.get().getX() - swerve.getState().Pose.getX())
                        + " meters of error x,"
                        + (targetSupplier.get().getY() - swerve.getState().Pose.getY())
                        + " meters of error y");
    }

    @Override
    public boolean isFinished() {
        return SwerveCommands.isAt(swerve, targetSupplier.get(), 0.1, 0.1).getAsBoolean();
    }
}
