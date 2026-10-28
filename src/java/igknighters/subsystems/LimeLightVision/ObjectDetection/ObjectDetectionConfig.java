package igknighters.subsystems.LimeLightVision.ObjectDetection;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * One Limelight running a neural detector pipeline, handed to {@link ObjectDetection}.
 *
 * <pre>
 * new ObjectDetectionConfig("limelight-back", 0.075, "fuel").mount(-0.31, -0.22, 0.20, 15, 180)
 * </pre>
 */
public class ObjectDetectionConfig {
    public final String name;

    /** Class names, index = class id (the order of the model's labels file). */
    public final String[] labels;

    /** Height of a detected object's center above the floor (meters), e.g. a ball's radius. */
    public final double objectHeight;

    /** Detections that land farther than this from the robot are dropped (meters). */
    public double maxObjectDistance = 6.0;

    /**
     * Frame-to-frame jitter of the detector's tx/ty (degrees, 1 std dev). Sets how much a far
     * detection is trusted. Measure it: log tync of a ball sitting still. The sim uses 0.3.
     */
    public double angleNoiseDeg = 0.3;

    /** Frames skipped while disabled, keeps the camera cool. */
    public int throttleDisabled = 300;

    /**
     * Robot-to-camera transform. Unlike localization, detection needs this on the robot too: it is
     * how a tx/ty becomes a spot on the floor. Keep it matching the mount in the Limelight web UI.
     */
    public Transform3d robotToCamera = Transform3d.kZero;

    public ObjectDetectionConfig(String name, double objectHeight, String... labels) {
        this.name = name;
        this.objectHeight = objectHeight;
        this.labels = labels;
    }

    public ObjectDetectionConfig maxObjectDistance(double v) {
        maxObjectDistance = v;
        return this;
    }

    public ObjectDetectionConfig angleNoiseDeg(double v) {
        angleNoiseDeg = v;
        return this;
    }

    public ObjectDetectionConfig throttleDisabled(int v) {
        throttleDisabled = v;
        return this;
    }

    /**
     * Mount from the Limelight's own robot-space numbers (web UI 3D tab, or pipeline json
     * rsf/rss/rsu/rspitch/rsyaw), same numbers as {@code CameraConfig.simMount}. Limelight "side"
     * is to the right and pitch is up, so both flip sign into WPILib.
     */
    public ObjectDetectionConfig mount(
            double forward, double side, double up, double pitchDeg, double yawDeg) {
        robotToCamera =
                new Transform3d(
                        forward,
                        -side,
                        up,
                        new Rotation3d(0, Math.toRadians(-pitchDeg), Math.toRadians(yawDeg)));
        return this;
    }
}
