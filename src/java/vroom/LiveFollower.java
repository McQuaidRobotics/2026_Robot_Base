package vroom;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import igknighters.Robot;
import igknighters.subsystems.swerve.Swerve;
import java.util.ArrayList;
import java.util.Set;
import vroom.Fields.EmptyField;

public class LiveFollower {
    private static SwerveRequest.FieldCentric m_driveRequest =
            new SwerveRequest.FieldCentric()
                    .withDeadband(
                            Robot.consts
                                            .swerve()
                                            .getCommonSwerveConsts()
                                            .getMaxSpeedMetersPerSecond()
                                    * 0.1)
                    .withRotationalDeadband(RotationsPerSecond.of(0.75).in(RadiansPerSecond) * .1)
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                    .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
                    .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

    private static PIDController thetaController = new PIDController(1.0, 0, 0);
    private static Field field = new EmptyField();

    private static Translation2d calculateTotalForce(Translation2d current, Translation2d target) {
        Translation2d attractive = target.minus(current);
        double distToTarget = attractive.getNorm();
        Translation2d unitAttractive =
                (distToTarget > 0) ? attractive.div(distToTarget) : new Translation2d();

        // Higher attraction weight helps "pull" the robot through narrow gaps
        Translation2d finalAttractive = unitAttractive.times(1);

        Translation2d totalRepulsive = new Translation2d();
        double robotRadius = 0.5;

        ArrayList<vroom.Obstacles.Obstacle> corridors = new ArrayList<>();

        for (vroom.Obstacles.Obstacle obs : field.getObstacles()) {
            if (obs instanceof vroom.Obstacles.CORIDOR) {
                corridors.add(obs);
                continue;
            }

            totalRepulsive =
                    totalRepulsive.plus(
                            obs.calculateForce(current, target, robotRadius, totalRepulsive));
        }

        Translation2d totalForce = finalAttractive.plus(totalRepulsive);
        // ensure corridors last
        for (vroom.Obstacles.Obstacle obs : corridors) {
            totalForce =
                    totalForce.plus(obs.calculateForce(current, target, robotRadius, totalForce));
        }
        return totalForce;
    }

    public static ChassisSpeeds calculateSpeeds(Pose2d current, Pose2d target) {
        // 1. Calculate the raw "Force Vector" based on current position
        // This uses your existing logic: Attraction + Repulsion
        Translation2d totalForce =
                calculateTotalForce(current.getTranslation(), target.getTranslation());

        // 2. Treat the force vector as your "Desired Velocity"
        // If the force is 1.0 meters, we want to go 1.0 m/s in that direction
        double desiredVx = totalForce.getX();
        double desiredVy = totalForce.getY();

        // 3. Use PID to "lock in" the velocity or smooth the movement
        // Since we are live, we usually PID the HEADING and just drive the XY
        double rotationOutput =
                thetaController.calculate(
                        current.getRotation().getRadians(), target.getRotation().getRadians());

        // 4. Scale to Max Speed
        // We don't want to exceed robot limits
        Translation2d driveVector = new Translation2d(desiredVx, desiredVy);
        if (driveVector.getNorm()
                > Robot.consts.swerve().getCommonSwerveConsts().getMaxSpeedMetersPerSecond()) {
            driveVector =
                    driveVector
                            .div(driveVector.getNorm())
                            .times(
                                    Robot.consts
                                            .swerve()
                                            .getCommonSwerveConsts()
                                            .getMaxSpeedMetersPerSecond());
        }

        return new ChassisSpeeds(driveVector.getX(), driveVector.getY(), rotationOutput);
    }

    public static Command driveLive(Swerve swerve, Pose2d target) {
        return Commands.defer(
                () ->
                        swerve.run(
                                () -> {
                                    ChassisSpeeds speeds =
                                            calculateSpeeds(swerve.getState().Pose, target);
                                    swerve.setControl(
                                            m_driveRequest
                                                    .withVelocityX(speeds.vxMetersPerSecond)
                                                    .withVelocityY(speeds.vyMetersPerSecond)
                                                    .withRotationalRate(
                                                            speeds.omegaRadiansPerSecond));
                                }),
                Set.of(swerve));
    }
}
