package igknighters.subsystems.LimeLightVision.Cameras;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import igknighters.Robot;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightHelpers;
import igknighters.util.Merging.PoseAverager;
import igknighters.util.log.Log;
import java.util.ArrayList;
import java.util.List;

public class LimeLightVisionReal extends LimeLights {

    private double previousSampleTime = 0.0;
    private final List<String> cameraNames;
    private double lastTimeStamp = 0.0;
    private final List<Integer> visibleTagIds = new ArrayList<>();
    // Read by the swerve telemetry thread, so it only ever sees a finished, immutable list
    private volatile List<Integer> visibleTagIdsSnapshot = List.of();

    public LimeLightVisionReal(String... cameraNames) {
        this.cameraNames = new ArrayList<>();
        for (String cameraName : cameraNames) {
            LimelightHelpers.SetIMUAssistAlpha(cameraName, 0.1);
            this.cameraNames.add(cameraName);
        }
    }

    /**
     * Returns a vision-based pose where translation comes from MT2 (reliable) and rotation comes
     * from MT1 (vision), ignoring MT1 translation entirely.
     */
    public Pose2d getRobotPoseFromVision(
            double yaw,
            double yawRate,
            double pitch,
            double pitchRate,
            double roll,
            double rollRate) {

        List<Pose2d> poses = new ArrayList<>();
        double timestampSum = 0.0;
        visibleTagIds.clear();
        // Feed gyro to every Limelight (for MT2), then flush NT once instead of once per camera
        for (String cameraName : cameraNames) {
            LimelightHelpers.SetRobotOrientation_NoFlush(
                    cameraName, yaw, yawRate, pitch, pitchRate, roll, rollRate);
        }
        LimelightHelpers.Flush();
        // mode breakdown
        // 1 = make internal match gyro
        for (String cameraName : cameraNames) {

            // Get both MT2 and MT1 estimates
            var mt2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
            var mt1Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

            if (mt2Estimate != null && mt1Estimate != null && mt1Estimate.tagCount > 0) {

                // --- ROTATION SELECTION LOGIC ---
                Rotation2d rotationToUse;
                if (mt1Estimate.tagCount >= 2) {
                    if (RobotController.getFPGATime() - previousSampleTime > 0.0) {
                        // only accept the newer ones. This makes it so that if the last camera
                        // is
                        // behind the others it will still work
                        previousSampleTime = RobotController.getFPGATime();
                    }
                    rotationToUse = mt1Estimate.pose.getRotation(); // vision rotation
                } else {
                    rotationToUse = mt2Estimate.pose.getRotation(); // fallback gyro-based
                }

                // MT2 translation + selected rotation
                Pose2d rotationOnlyPose =
                        new Pose2d(mt2Estimate.pose.getTranslation(), rotationToUse);

                poses.add(rotationOnlyPose);

                // accumulate timestamp
                timestampSum += mt2Estimate.timestampSeconds;

                // collect visible tags
                for (var fiducial : mt2Estimate.rawFiducials) {
                    visibleTagIds.add(fiducial.id);
                }

                // Optional: log rotation source
                if (!Robot.consts.limelightVision().disableVisionLogs()) {
                    Log.log(
                            "Subsystems/Vision/LimeLightVision/Source_" + cameraName,
                            (mt1Estimate.tagCount >= 2) ? "VISION_CORRECTION" : "ROBOT_GYRO_ONLY");
                }
            }

            double timestamp = !poses.isEmpty() ? timestampSum / poses.size() : 0.0;
            lastTimeStamp = timestamp;

            if (!Robot.consts.limelightVision().disableVisionLogs()) {
                Log.log(
                        "ROBOT/Subsystems/Vision/LimeLightVision/TimeStampOfMeasurements",
                        timestamp);
                Log.log(
                        "ROBOT/Subsystems/Vision/LimeLightVision/NumberOfTagsSeen",
                        visibleTagIds.size());
            }
        }

        double timestamp = !poses.isEmpty() ? timestampSum / poses.size() : 0.0;
        lastTimeStamp = timestamp;

        if (!Robot.consts.limelightVision().disableVisionLogs()) {
            Log.log("ROBOT/Subsystems/Vision/LimeLightVision/TimeStampOfMeasurements", timestamp);
            Log.log(
                    "ROBOT/Subsystems/Vision/LimeLightVision/NumberOfTagsSeen",
                    visibleTagIds.size());
        }

        visibleTagIdsSnapshot = List.copyOf(visibleTagIds);
        return PoseAverager.averagePose2ds(poses);
    }

    @Override
    public void saveCameras() {
        // disabled
        for (String cameraName : cameraNames) {
            LimelightHelpers.SetThrottle(cameraName, 300);
            LimelightHelpers.SetIMUMode(cameraName, 1);
        }
    }

    /** Returns a list of visible tag IDs in the current frame. */
    public List<Integer> getVisibleTagIds() {
        return visibleTagIdsSnapshot;
    }

    public double timeSinceLastSample() {
        return (RobotController.getFPGATime() - previousSampleTime)
                * (1 / 1000000.0); // microseconds to seconds
    }

    @Override
    public void enableCameras(int imu_mode) {
        for (String name : cameraNames) {
            LimelightHelpers.SetThrottle(name, 0);
            LimelightHelpers.SetIMUMode(name, imu_mode);
        }
    }

    /** Returns the last timestamp from vision measurements. */
    public double getLastTimeStamp() {
        return lastTimeStamp;
    }
}
