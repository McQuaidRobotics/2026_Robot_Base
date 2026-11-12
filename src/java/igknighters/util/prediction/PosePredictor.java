package igknighters.util.prediction;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.FieldVisualizer;
import igknighters.subsystems.swerve.Swerve;
import java.util.ArrayList;

public class PosePredictor extends SubsystemBase {
    ArrayList<Sample> Xposes;
    ArrayList<Sample> Yposes;
    ArrayList<Sample> Wposes;
    Swerve swerve;
    Derivative derivative;

    public PosePredictor(Swerve swerve) {
        this.Xposes = new ArrayList<Sample>();
        this.Yposes = new ArrayList<Sample>();
        this.Wposes = new ArrayList<Sample>();
        this.derivative = new Derivative();
        this.swerve = new Swerve();
    }

    public void updatePoints(Swerve swerve) {
        Sample Xpoint =
                new Sample(
                        swerve.getState().Speeds.vxMetersPerSecond,
                        RobotController.getMeasureFPGATime());
        Xposes.add(Xpoint);

        if (Xposes.size() > 10) {
            Xposes.remove(0);
        }
        Sample Ypoint =
                new Sample(
                        swerve.getState().Speeds.vyMetersPerSecond,
                        RobotController.getMeasureFPGATime());
        Yposes.add(Ypoint);
        if (Yposes.size() > 10) {
            Yposes.remove(0);
        }
        Sample Wpoint =
                new Sample(
                        swerve.getState().Speeds.omegaRadiansPerSecond,
                        RobotController.getMeasureFPGATime());
        Wposes.add(Wpoint);
        if (Wposes.size() > 10) {
            Wposes.remove(0);
        }
    }

    public Pose2d getPredictedPose(Time time, Swerve swerve) {
        SwerveDriveState state = swerve.getState();

        ChassisSpeeds speedFieldRelative = swerve.getFieldRelativeSpeeds();

        double posex = state.Pose.getX();
        double posey = state.Pose.getY();
        double posew = state.Pose.getRotation().getRadians();

        double speedx = speedFieldRelative.vxMetersPerSecond;
        double speedy = speedFieldRelative.vyMetersPerSecond;
        double speedw = speedFieldRelative.omegaRadiansPerSecond;

        double x =
                posex
                        + (speedx * time.in(Seconds)
                                + (.5
                                        * derivative.makeDerivative(Xposes)
                                        * time.in(Seconds)
                                        * time.in(Seconds)));
        double y =
                posey
                        + (speedy * time.in(Seconds)
                                + (.5
                                        * derivative.makeDerivative(Yposes)
                                        * time.in(Seconds)
                                        * time.in(Seconds)));
        double w =
                posew
                        + (speedw * time.in(Seconds)
                                + (.5
                                        * derivative.makeDerivative(Wposes)
                                        * time.in(Seconds)
                                        * time.in(Seconds)));
        Pose2d pred_pose = new Pose2d(x, y, new Rotation2d(w));
        return pred_pose;
    }

    @Override
    public void periodic() {
        updatePoints(swerve);
        Pose2d targetPose = getPredictedPose(Milliseconds.of(100), swerve);
        FieldVisualizer.getInstance().updateDrivingTarget(targetPose);
        super.periodic();
    }
}
