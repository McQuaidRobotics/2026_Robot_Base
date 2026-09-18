package igknighters.subsystems.LimeLightVision.Cameras;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import igknighters.Robot;
import igknighters.subsystems.LimeLightVision.CameraData;
import igknighters.subsystems.LimeLightVision.CameraData.Pipelines;
import limelight.Limelight;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.PoseEstimate;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.LEDMode;

public class YallLimelight {
    public CameraData data;
    public Limelight camera;
    public LimelightPoseEstimator poseEstimator;
    public boolean rotation_modified = false;
    public Pipelines functioning_as_pipeline;


    public YallLimelight(CameraData data) {
        this.data = data;
        camera = new Limelight(data.name);
        camera.getSettings().withLimelightLEDMode(LEDMode.PipelineControl).withCameraOffset(data.cameraFloorRobotCenter).save();
        poseEstimator = camera.createPoseEstimator(EstimationMode.MEGATAG2);
        

        switch (data.cameraPipeline) {
            case POSE_DETECTION:
                camera.getSettings().withPipelineIndex(1);
                break;
            case OBJECT_DETECTION:
                camera.getSettings().withPipelineIndex(2);
                break;
            case TAG_TRACKING:
                camera.getSettings().withPipelineIndex(3);
                break;
            case DOES_EVERYTHING:
                break;
            default:
                DriverStation.reportWarning("THE CAMERA NAMED: " + data.name + " is not configured to be any kind of known pipeline", true);
                break;
        }
    }
    /**
     * This is the method for a singular static camera for pose finding if it is on a turret or moving you must supply the angle of the turret in periodic.
     * @return null if not valid and the pose reported by the camera if valid
     */
    public Pose2d getRobotPoseFromVision() {
        if (!(data.cameraPipeline.equals(Pipelines.DOES_EVERYTHING) | data.cameraPipeline.equals(Pipelines.POSE_DETECTION))) {
            DriverStation.reportWarning("YOU ARE ASKING FOR A POSE FROM A NON POSE DESIGNED CAMERA", null);
        }
        // non zero offset but you havent told anything its rotation so things will be wrong. Which is why you need to tell it the rotation
        if (!data.cameraOffsetFromAxisOfRotation.equals(null) && rotation_modified == false) {
            DriverStation.reportWarning("YOU ARE NOT SUPLYING A ROTATION TO A ROTATING CAMERA THIS WILL MESS UP VISION MEAUREMENTS", true);
            return null;
        } else {
            Optional<PoseEstimate> visionEstimate = poseEstimator.getPoseEstimate();
            // if no tag then it will not be present
            if (visionEstimate.isPresent()) {
                PoseEstimate validEstimate = visionEstimate.get();
                // criteria for a valid detection
                // 1. Under 4 m
                // 2. more then 1 tag
                // 3. Low ish ambiguity (Im not quite sure what this means exactly but its in YALL Docs)
                if (validEstimate.avgTagDist < 4
                    && validEstimate.tagCount > 1
                    && validEstimate.getMinTagAmbiguity() < .3) {
                        return validEstimate.pose.toPose2d();
                    }
                else {
                    // does not meet detection requirments
                    return null;
                }
            } else {
                // no measurement present
                return null;
            }
        }
    }


    /**
     * MUST BE CALLED EVERY CYCLE FOR STATIC LLS DO NOT CALL PERIODIC two times. This will update the LL orientation.
     */
    public void periodic() {
        // Must be called every cycle by the manager
        camera.getSettings().withRobotOrientation(Robot.robotOrientation).save();
    }
    /**
     * THIS IS FOR ROTATIONAL ONLY DO NOT CALL ON LLS THAT DO NOT MOVE RELATIVE TO ROBOT
     * @param rotation3d ROTATION RELATIVE TO ROBOT
     */
    public void periodic(Rotation3d rotation3d) {
        // Must be called every cycle by manager
        rotation_modified = true;

        camera.getSettings().withRobotOrientation(Robot.robotOrientation).withCameraOffset(new Pose3d(data.cameraOffsetFromAxisOfRotation.getTranslation(), rotation3d)).save();
    }




}
