package igknighters.subsystems.LimeLightVision.Localization;

import edu.wpi.first.math.geometry.Pose3d;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightHelpers;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightHelpers.IMUData;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightHelpers.LimelightResults;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightHelpers.PoseEstimate;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightHelpers.RawDetection;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightHelpers.RawFiducial;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightHelpers.RawTarget;

/**
 * One physical Limelight. Raw passthrough to LimelightHelpers: no filtering, no math, no state.
 *
 * <p>Anything that wants camera data (pose fusion, object detection, tag tracking, ...) grabs the
 * camera from {@link LocalizationVision#camera(String)} and calls these directly. Units and frames
 * are whatever the Limelight publishes (degrees, meters, wpiBlue unless named otherwise).
 */
public class Limelight {
    public final String name;

    public Limelight(String name) {
        this.name = name;
    }

    // ---------------- Localization ----------------

    /** MegaTag1 (vision only, solves heading itself). Never null, tagCount == 0 when no tags. */
    public PoseEstimate getMegaTag1() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    }

    /** MegaTag2 (uses heading from {@link #setRobotOrientation}). Never null. */
    public PoseEstimate getMegaTag2() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    }

    public PoseEstimate getMegaTag1Red() {
        return LimelightHelpers.getBotPoseEstimate_wpiRed(name);
    }

    public PoseEstimate getMegaTag2Red() {
        return LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(name);
    }

    /** Limelight's own std devs: [MT1 x,y,z,roll,pitch,yaw, MT2 x,y,z,roll,pitch,yaw]. */
    public double[] getStdDevs() {
        return LimelightHelpers.getLimelightNTDoubleArray(name, "stddevs");
    }

    public Pose3d getBotPose3d() {
        return LimelightHelpers.getBotPose3d(name);
    }

    public Pose3d getBotPose3dBlue() {
        return LimelightHelpers.getBotPose3d_wpiBlue(name);
    }

    public Pose3d getBotPose3dRed() {
        return LimelightHelpers.getBotPose3d_wpiRed(name);
    }

    public Pose3d getBotPose3dTargetSpace() {
        return LimelightHelpers.getBotPose3d_TargetSpace(name);
    }

    public Pose3d getCameraPose3dTargetSpace() {
        return LimelightHelpers.getCameraPose3d_TargetSpace(name);
    }

    public Pose3d getCameraPose3dRobotSpace() {
        return LimelightHelpers.getCameraPose3d_RobotSpace(name);
    }

    public Pose3d getTargetPose3dCameraSpace() {
        return LimelightHelpers.getTargetPose3d_CameraSpace(name);
    }

    public Pose3d getTargetPose3dRobotSpace() {
        return LimelightHelpers.getTargetPose3d_RobotSpace(name);
    }

    // ---------------- Targets (tags, detector, classifier, color, barcode) ----------------

    public boolean hasTarget() {
        return LimelightHelpers.getTV(name);
    }

    public double getTX() {
        return LimelightHelpers.getTX(name);
    }

    public double getTY() {
        return LimelightHelpers.getTY(name);
    }

    public double getTXNC() {
        return LimelightHelpers.getTXNC(name);
    }

    public double getTYNC() {
        return LimelightHelpers.getTYNC(name);
    }

    public double getTA() {
        return LimelightHelpers.getTA(name);
    }

    public double[] getT2D() {
        return LimelightHelpers.getT2DArray(name);
    }

    public int getTargetCount() {
        return LimelightHelpers.getTargetCount(name);
    }

    /** Primary AprilTag id, -1 when none. */
    public int getFiducialID() {
        return (int) LimelightHelpers.getFiducialID(name);
    }

    public RawFiducial[] getRawFiducials() {
        return LimelightHelpers.getRawFiducials(name);
    }

    public RawDetection[] getRawDetections() {
        return LimelightHelpers.getRawDetections(name);
    }

    public RawTarget[] getRawTargets() {
        return LimelightHelpers.getRawTargets(name);
    }

    public double[] getCornerCoordinates() {
        return LimelightHelpers.getCornerCoordinates(name);
    }

    public int getDetectorClassIndex() {
        return LimelightHelpers.getDetectorClassIndex(name);
    }

    public String getDetectorClass() {
        return LimelightHelpers.getDetectorClass(name);
    }

    public int getClassifierClassIndex() {
        return LimelightHelpers.getClassifierClassIndex(name);
    }

    public String getClassifierClass() {
        return LimelightHelpers.getClassifierClass(name);
    }

    public double[] getTargetColor() {
        return LimelightHelpers.getTargetColor(name);
    }

    public String[] getBarcodeData() {
        return LimelightHelpers.getRawBarcodeData(name);
    }

    public double[] getPythonScriptData() {
        return LimelightHelpers.getPythonScriptData(name);
    }

    /** Full JSON parse. Slow (several ms), do not call every loop. */
    public LimelightResults getLatestResults() {
        return LimelightHelpers.getLatestResults(name);
    }

    public String getJSONDump() {
        return LimelightHelpers.getJSONDump(name);
    }

    // ---------------- Health ----------------

    /** Increments once per processed frame. Stops changing when the camera is unplugged. */
    public double getHeartbeat() {
        return LimelightHelpers.getHeartbeat(name);
    }

    /** [fps, cpu temp, ram usage, temp]. */
    public double[] getHardwareStats() {
        return LimelightHelpers.getLimelightNTDoubleArray(name, "hw");
    }

    public double getPipelineLatencyMs() {
        return LimelightHelpers.getLatency_Pipeline(name);
    }

    public double getCaptureLatencyMs() {
        return LimelightHelpers.getLatency_Capture(name);
    }

    public int getPipelineIndex() {
        return (int) LimelightHelpers.getCurrentPipelineIndex(name);
    }

    public String getPipelineType() {
        return LimelightHelpers.getCurrentPipelineType(name);
    }

    public IMUData getIMUData() {
        return LimelightHelpers.getIMUData(name);
    }

    // ---------------- Setters ----------------

    /** Heading for MegaTag2. Degrees and deg/s, wpiBlue. Does not flush, the caller does once. */
    public void setRobotOrientation(double yawDeg, double yawRateDps) {
        LimelightHelpers.SetRobotOrientation_NoFlush(name, yawDeg, yawRateDps, 0, 0, 0, 0);
    }

    /** Camera mount relative to robot center. Meters and degrees. For turreted/moving cameras. */
    public void setCameraPoseRobotSpace(
            double forward, double side, double up, double roll, double pitch, double yaw) {
        LimelightHelpers.setCameraPose_RobotSpace(name, forward, side, up, roll, pitch, yaw);
    }

    public void setPipelineIndex(int index) {
        LimelightHelpers.setPipelineIndex(name, index);
    }

    public void setPriorityTagID(int id) {
        LimelightHelpers.setPriorityTagID(name, id);
    }

    /** Limit which tags are used for localization. */
    public void setTagFilter(int... ids) {
        LimelightHelpers.SetFiducialIDFiltersOverride(name, ids);
    }

    /** 0 = pipeline control, 1.5/2/3/4 = forced downscale. */
    public void setTagDownscale(float downscale) {
        LimelightHelpers.SetFiducialDownscalingOverride(name, downscale);
    }

    public void setFiducial3DOffset(double x, double y, double z) {
        LimelightHelpers.setFiducial3DOffset(name, x, y, z);
    }

    /**
     * 0 external, 1 seed internal from external, 2 internal, 3 internal+MT1 assist, 4
     * internal+external assist.
     */
    public void setIMUMode(int mode) {
        LimelightHelpers.SetIMUMode(name, mode);
    }

    public void setIMUAssistAlpha(double alpha) {
        LimelightHelpers.SetIMUAssistAlpha(name, alpha);
    }

    /** Skip this many frames between processed frames. 0 = full speed. */
    public void setThrottle(int skipFrames) {
        LimelightHelpers.SetThrottle(name, skipFrames);
    }

    public void setCropWindow(double xMin, double xMax, double yMin, double yMax) {
        LimelightHelpers.setCropWindow(name, xMin, xMax, yMin, yMax);
    }

    public void setKeystone(double horizontal, double vertical) {
        LimelightHelpers.setKeystone(name, horizontal, vertical);
    }

    public void setLEDPipelineControl() {
        LimelightHelpers.setLEDMode_PipelineControl(name);
    }

    public void setLEDOff() {
        LimelightHelpers.setLEDMode_ForceOff(name);
    }

    public void setLEDBlink() {
        LimelightHelpers.setLEDMode_ForceBlink(name);
    }

    public void setLEDOn() {
        LimelightHelpers.setLEDMode_ForceOn(name);
    }

    public void setStreamStandard() {
        LimelightHelpers.setStreamMode_Standard(name);
    }

    public void setStreamPiPMain() {
        LimelightHelpers.setStreamMode_PiPMain(name);
    }

    public void setStreamPiPSecondary() {
        LimelightHelpers.setStreamMode_PiPSecondary(name);
    }

    public void setPythonScriptData(double... data) {
        LimelightHelpers.setPythonScriptData(name, data);
    }

    public void triggerSnapshot() {
        LimelightHelpers.triggerSnapshot(name);
    }

    public void setRewindEnabled(boolean enabled) {
        LimelightHelpers.setRewindEnabled(name, enabled);
    }

    public void triggerRewindCapture(double seconds) {
        LimelightHelpers.triggerRewindCapture(name, seconds);
    }
}
