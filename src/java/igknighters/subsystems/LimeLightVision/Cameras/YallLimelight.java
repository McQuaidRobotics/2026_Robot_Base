package igknighters.subsystems.LimeLightVision.Cameras;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import igknighters.Robot;
import igknighters.subsystems.LimeLightVision.CameraData;
import igknighters.subsystems.LimeLightVision.CameraData.Pipelines;
import igknighters.subsystems.LimeLightVision.LimeLightVision;
import igknighters.subsystems.LimeLightVision.LimeLightVision.object_output;
import igknighters.subsystems.LimeLightVision.LimeLightVision.pose_output;
import igknighters.subsystems.LimeLightVision.LimeLightVision.tag_output;
import java.util.ArrayList;
import java.util.Optional;
import limelight.Limelight;
import limelight.networktables.LimelightData;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightResults;
import limelight.networktables.LimelightSettings.ImuMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.PoseEstimate;
import limelight.networktables.target.pipeline.NeuralDetector;
import limelight.results.RawFiducial;
import limelight.sim.LimelightSim;

public class YallLimelight {
    public CameraData data;
    public Limelight camera;
    public LimelightSim sim_camera;
    public LimelightPoseEstimator poseEstimator;
    public boolean rotation_modified = false;
    public Pipelines functioning_as_pipeline;

    public YallLimelight(CameraData data) {
        this.data = data;
        camera = new Limelight(data.name);
        sim_camera = new LimelightSim(camera);
        sim_camera.withField2d(LimeLightVision.field_for_testing);
        // WE NEED TO REFACTOR FIELD_VISUALIZER TO USE A FIELD 2d SO THAT WE CAN DO
        // sim_camera.withField()
        if (data.cameraFloorRobotCenter.equals(null)) {
            // the sim will assume its at the center of the robot and pointing forward in this state
            // so carefull with sim. Better to have configs in code anyway
            camera.getSettings().withLimelightLEDMode(LEDMode.PipelineControl).save();
        } else {
            camera.getSettings()
                    .withLimelightLEDMode(LEDMode.PipelineControl)
                    .withCameraOffset(data.cameraFloorRobotCenter)
                    .save();
        }
        poseEstimator = camera.createPoseEstimator(EstimationMode.MEGATAG2);

        switch (data.cameraPipeline) {
            case POSE_DETECTION:
                camera.getSettings().withPipelineIndex(1).save();
                break;
            case OBJECT_DETECTION:
                camera.getSettings().withPipelineIndex(2).save();
                break;
            case TAG_TRACKING:
                camera.getSettings().withPipelineIndex(3).save();
                break;
            case DOES_EVERYTHING:
                break;
            default:
                DriverStation.reportWarning(
                        "THE CAMERA NAMED: "
                                + data.name
                                + " is not configured to be any kind of known pipeline",
                        true);
                break;
        }
    }

    public double calculateScore(double ambiguity, double distanceToRobot) {
        return (2 - (ambiguity + (distanceToRobot / 4.0)))
                / 2.0; // 4 is the max range im allowing for a comfortable detection so 0 ambiguity
        // at 4 m is 50% confident
    }

    public void setThrottle(double throttle) {
        camera.getSettings().withThrottle(throttle).save();
    }

    public void setIMUMode(ImuMode imuMode) {
        camera.getSettings().withImuMode(imuMode);
    }

    /**
     * This is the method for a singular static camera for pose finding if it is on a turret or
     * moving you must supply the angle of the turret in periodic.
     *
     * @return null if not valid and the pose reported by the camera if valid
     */
    public pose_output getRobotPoseFromVision() {
        if (!(data.cameraPipeline.equals(Pipelines.DOES_EVERYTHING)
                | data.cameraPipeline.equals(Pipelines.POSE_DETECTION))) {
            DriverStation.reportWarning(
                    "YOU ARE ASKING FOR A POSE FROM A NON POSE DESIGNED CAMERA", null);
            return null;
        }
        if (data.cameraPipeline.equals(Pipelines.DOES_EVERYTHING)
                && !functioning_as_pipeline.equals(Pipelines.POSE_DETECTION)) {
            functioning_as_pipeline = Pipelines.POSE_DETECTION;
            camera.getSettings().withPipelineIndex(1).save();
        }
        // non zero offset but you havent told anything its rotation so things will be wrong. Which
        // is why you need to tell it the rotation
        if (!data.cameraOffsetFromAxisOfRotation.equals(null) && rotation_modified == false) {
            DriverStation.reportWarning(
                    "YOU ARE NOT SUPLYING A ROTATION TO A ROTATING CAMERA THIS WILL MESS UP VISION"
                            + " MEAUREMENTS",
                    true);
            return null;
        } else {
            Optional<PoseEstimate> visionEstimate = poseEstimator.getPoseEstimate();
            // if no tag then it will not be present
            if (visionEstimate.isPresent()) {
                PoseEstimate validEstimate = visionEstimate.get();
                // criteria for a valid detection
                // 1. Under 4 m
                // 2. more then 1 tag
                // 3. Low ish ambiguity (Im not quite sure what this means exactly but its in YALL
                // Docs)
                if (validEstimate.avgTagDist < 4
                        && validEstimate.tagCount > 1
                        && validEstimate.getMinTagAmbiguity() < .3) {
                    return new pose_output(
                            validEstimate.pose.toPose2d(),
                            Microseconds.of(RobotController.getFPGATime()));
                } else {
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
     * Get tag translation and area based off of the id you care about and the pickyness you set
     *
     * @param tag_id
     * @param pickyness is a parameter that will determine how picky the camera is with filtering
     *     tags 0 is allow everything 1 is deny everything
     * @return a translation 3d where x = tx y = ty and z = ta or null
     */
    public tag_output getTagTranslation(Integer tag_id, double pickyness) {
        if (!(data.cameraPipeline.equals(Pipelines.DOES_EVERYTHING)
                | data.cameraPipeline.equals(Pipelines.TAG_TRACKING))) {
            DriverStation.reportWarning(
                    "YOU ARE ASKING FOR A TAG INFO FROM A NON TAG DESIGNED CAMERA", null);
            return null;
        }
        if (data.cameraPipeline.equals(Pipelines.DOES_EVERYTHING)
                && !functioning_as_pipeline.equals(Pipelines.TAG_TRACKING)) {
            functioning_as_pipeline = Pipelines.TAG_TRACKING;
            camera.getSettings().withPipelineIndex(3).save();
        }
        LimelightData data = camera.getData();
        for (RawFiducial tag : data.getRawFiducials()) {
            // tag.id, tag.txnc, tag.tync, tag.ta
            // tag.distToCamera, tag.distToRobot (meters)
            // tag.ambiguity (0-1, lower is more trustworthy)

            if (tag.id == tag_id) {
                if (calculateScore(tag.ambiguity, tag.distToCamera) >= pickyness) {

                    return new tag_output(
                            new Translation3d(tag.txnc, tag.tync, tag.ta),
                            Milliseconds.of(RobotController.getFPGATime()));
                }
            }
        }
        return null;
    }

    public object_output getObjectTranslation(String objectName, double required_confidence) {
        if (!(data.cameraPipeline.equals(Pipelines.DOES_EVERYTHING)
                | data.cameraPipeline.equals(Pipelines.OBJECT_DETECTION))) {
            DriverStation.reportWarning(
                    "YOU ARE ASKING FOR A TAG INFO FROM A NON TAG DESIGNED CAMERA", null);
            return null;
        }
        if (data.cameraPipeline.equals(Pipelines.DOES_EVERYTHING)
                && !functioning_as_pipeline.equals(Pipelines.OBJECT_DETECTION)) {
            functioning_as_pipeline = Pipelines.TAG_TRACKING;
            camera.getSettings().withPipelineIndex(2).save();
        }

        Optional<LimelightResults> potential_results = camera.getLatestResults();

        if (!potential_results.isPresent()) {
            return null;
        }

        LimelightResults results = potential_results.get();
        ArrayList<Translation3d> outputs = new ArrayList<Translation3d>();
        for (NeuralDetector object :
                results.targets_Detector) { // target detector is individual boxes around obj
            // classifier is whole frame eg is frame a coral
            if (object.className.equals(objectName) && object.confidence > required_confidence) {
                outputs.add(new Translation3d(object.tx, object.ty, object.ta));
            }
        }

        if (outputs.isEmpty()) {
            return null;
        } else {
            return new object_output(outputs, Microseconds.of(RobotController.getFPGATime()));
        }
    }

    /**
     * MUST BE CALLED EVERY CYCLE FOR STATIC LLS DO NOT CALL PERIODIC two times. This will update
     * the LL orientation.
     */
    public void periodic() {
        // Must be called every cycle by the manager
        camera.getSettings().withRobotOrientation(Robot.robotOrientation).save();
    }

    /**
     * THIS IS FOR ROTATIONAL ONLY DO NOT CALL ON LLS THAT DO NOT MOVE RELATIVE TO ROBOT
     *
     * @param rotation3d ROTATION RELATIVE TO ROBOT
     */
    public void periodic(Rotation3d rotation3d) {
        // Must be called every cycle by manager
        rotation_modified = true;

        camera.getSettings()
                .withRobotOrientation(Robot.robotOrientation)
                .withCameraOffset(
                        new Pose3d(
                                data.cameraOffsetFromAxisOfRotation.getTranslation(), rotation3d))
                .save();
    }

    public void simPeriodic(Pose2d robotPose) {
        sim_camera.update(robotPose);
    }
}
