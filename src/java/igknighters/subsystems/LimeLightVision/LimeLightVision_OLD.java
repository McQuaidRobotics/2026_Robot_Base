package igknighters.subsystems.LimeLightVision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import igknighters.constants.SubsystemConstants;
import igknighters.subsystems.LimeLightVision.Cameras.LimeLightVisionReal;
import igknighters.subsystems.LimeLightVision.Cameras.LimeLightVisionSim;
import igknighters.subsystems.LimeLightVision.Cameras.LimeLights;
import igknighters.util.log.Log;

import java.util.ArrayList;
import java.util.List;

public class LimeLightVision_OLD extends SubsystemBase {
    private LimeLights vision;

    public record pose_output(Pose2d pose, Time time){}
    public record tag_output(Translation3d tag_info, Time time){}
    public record object_output(ArrayList<Translation3d> object_info, Time time){}

    public LimeLightVision_OLD() {
        if (Robot.isReal()) {
            vision =
                    new LimeLightVisionReal(
                            SubsystemConstants.kLimelightVision.backCam,
                            SubsystemConstants.kLimelightVision.rightCam,
                            SubsystemConstants.kLimelightVision.primaryCam,
                            SubsystemConstants.kLimelightVision.frontCam);
        } else {
            vision = new LimeLightVisionSim("1", "2", "3", "4");
        }
    }

    public List<Integer> getVisibleTagIds() {
        return vision.getVisibleTagIds();
    }

    public void enableCameras(int IMU_MODE) {
        vision.enableCameras(IMU_MODE);
    }

    public void disableCameras() {
        vision.saveCameras();
    }

    public double getLastTimeStamp() {
        return vision.getLastTimeStamp();
    }

    public double timeSinceLastSample() {
        return vision.timeSinceLastSample();
    }

    public Pose2d getRobotPoseFromVision(
            double yaw,
            double yawRate,
            double pitch,
            double pitchRate,
            double roll,
            double rollRate) {
        if (!Robot.consts.limelightVision().disableVisionLogs()) {
            Log.log("ROBOT/Subsystems/Vison/Limelight/ENABLED", true);
        }
        return vision.getRobotPoseFromVision(yaw, yawRate, pitch, pitchRate, roll, rollRate);
    }
}
