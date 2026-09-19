package igknighters.subsystems.LimeLightVision;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import igknighters.constants.SubsystemConstants.kLimelightVision;
import igknighters.subsystems.LimeLightVision.CameraData.Pipelines;
import igknighters.subsystems.LimeLightVision.Cameras.YallLimelight;
import igknighters.util.log.Log;
import java.util.ArrayList;
import java.util.List;
import limelight.networktables.LimelightSettings.ImuMode;

public class LimeLightVision extends SubsystemBase {
    public static Field2d field_for_testing = new Field2d();

    public record pose_output(Pose2d pose, Time time) {}

    public record tag_output(Translation3d tag_info, Time time) {}

    public record object_output(ArrayList<Translation3d> object_info, Time time) {}

    ArrayList<YallLimelight> cameras = new ArrayList<YallLimelight>();

    public Time latestMeasurementTime = Seconds.of(0.0);

    public LimeLightVision() {
        cameras.add(
                new YallLimelight(
                        new CameraData(
                                kLimelightVision.frontCam,
                                new Pose3d(new Translation3d(), new Rotation3d(0, 0, 0)),
                                Pipelines.POSE_DETECTION)));
        cameras.add(
                new YallLimelight(
                        new CameraData(
                                kLimelightVision.backCam,
                                new Pose3d(new Translation3d(), new Rotation3d(0, 0, 3.14)),
                                Pipelines.POSE_DETECTION)));
        cameras.add(
                new YallLimelight(
                        new CameraData(
                                kLimelightVision.rightCam,
                                new Pose3d(
                                        new Translation3d(), new Rotation3d(0, 0, 3 * 3.14 / 2.0)),
                                Pipelines.POSE_DETECTION)));
        cameras.add(
                new YallLimelight(
                        new CameraData(
                                kLimelightVision.primaryCam,
                                new Pose3d(new Translation3d(), new Rotation3d(0, 0, 3.14 / 2.0)),
                                Pipelines.POSE_DETECTION)));
    }

    public List<Integer> getVisibleTagIds() {
        // ONLY NON WORKING FEATURE. NOT SURE HOW TO IMPLEMENT WITHOUT MAKING DUPLICATE CALS
        return new ArrayList<Integer>();
    }

    public void enableCameras(ImuMode IMU_MODE) {
        for (YallLimelight camera : cameras) {
            camera.setIMUMode(IMU_MODE);
            camera.setThrottle(0);
        }
    }

    public void disableCameras() {
        for (YallLimelight camera : cameras) {
            camera.setThrottle(50);
        }
    }

    public double getLastTimeStamp() {
        return latestMeasurementTime.in(Seconds);
    }

    public double timeSinceLastSample() {
        return RobotController.getFPGATime() / 1e6 - latestMeasurementTime.in(Seconds);
    }

    public List<pose_output> getRobotPoseFromVision(
            double yaw,
            double yawRate,
            double pitch,
            double pitchRate,
            double roll,
            double rollRate) {
        if (!Robot.consts.limelightVision().disableVisionLogs()) {
            Log.log("ROBOT/Subsystems/Vison/Limelight/ENABLED", true);
        }
        ArrayList<pose_output> outputs = new ArrayList<pose_output>();
        Time max = Seconds.of(0.0);
        for (YallLimelight camera : cameras) {
            if (camera.data.cameraPipeline.equals(Pipelines.POSE_DETECTION)) {
                pose_output output = camera.getRobotPoseFromVision();
                if (output.equals(null)) {
                    continue;
                }
                if (output.time.minus(max).in(Seconds) >= 0.0) {
                    max = output.time;
                }
                outputs.add(output);
            }
        }
        latestMeasurementTime = max;
        return outputs;
    }

    @Override
    public void periodic() {
        for (YallLimelight limelight : cameras) {
            limelight.periodic();
        }
    }

    @Override
    public void simulationPeriodic() {
        for (YallLimelight limelight : cameras) {
            limelight.simPeriodic(Robot.pose_pred.getDynamicPredictedPose());
        }
    }
}
