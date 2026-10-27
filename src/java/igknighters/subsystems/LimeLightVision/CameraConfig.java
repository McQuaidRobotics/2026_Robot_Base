package igknighters.subsystems.LimeLightVision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;

/**
 * Per-camera settings handed to {@link LimeLightVision}. Everything that differs between cameras or
 * robots lives here so the vision code itself never has to change.
 *
 * <pre>
 * new CameraConfig("limelight-intake").localize(false)          // object detection only
 * new CameraConfig("limelight-back").xyStdDevCoeff(0.04)         // trust this one less
 * new CameraConfig("limelight-turret").mount(turret::cameraPose) // camera that moves
 * </pre>
 *
 * Std dev model: stdDev = coeff * avgTagDist^2 / tagCount (meters, radians).
 */
public class CameraConfig {
    public final String name;

    /** Feed this camera's pose into the drivetrain estimator. */
    public boolean localize = true;

    // The estimator moves a fixed fraction toward EVERY frame and we get ~150 frames/s (3 cams), so
    // per-frame std devs must be much larger than one frame's real error. 0.02/0.06 jittered ~3 cm
    // and 0.5 deg in sim; these give ~1.4 cm / 0.2 deg and still fix a 1 m error in ~0.13 s.
    public double xyStdDevCoeff = 0.1;
    public double thetaStdDevCoeff = 0.3;

    /** Ignore frames whose average tag distance is farther than this (meters). */
    public double maxTagDistance = 5.0;

    /** MegaTag1 heading is only trusted when every tag's ambiguity is below this. */
    public double maxAmbiguity = 0.3;

    /** MT1 vs MT2 translation gap (meters) that means our heading is wrong. */
    public double maxMt1Mt2Disagreement = 1.0;

    /**
     * Degrees. Within this, MegaTag1 gently nudges our heading every frame. Beyond it, our heading
     * is wrong enough to bend MegaTag2's position, so MT2 position is not used and only the
     * vote-based heading reset may fix it. MT1 heading noise was ~1 deg on the robot.
     */
    public double maxHeadingCorrectionDeg = 5;

    /**
     * MegaTag1 must hold the same offset from our heading (within 5 degrees) this long before it
     * may reset a wrong heading (seconds). Works while turning, only flips break it.
     */
    public double headingResetStableSeconds = 1.0;

    public int imuModeEnabled = 0;
    public int imuModeDisabled = 1;

    /** Frames skipped while disabled, keeps the camera cool. */
    public int throttleDisabled = 300;

    /** Robot-to-camera pose pushed every loop. null = fixed mount set in the Limelight web UI. */
    public Supplier<Pose3d> mount = null;

    /**
     * Sim only: robot-to-camera transform for the camera simulator. On the robot the mount lives in
     * the Limelight itself (limelight-configs/&lt;name&gt;/pipeline_0.json rsf/rss/rsu/rs*).
     */
    public Transform3d simMount = Transform3d.kZero;

    public CameraConfig(String name) {
        this.name = name;
    }

    public CameraConfig localize(boolean v) {
        localize = v;
        return this;
    }

    public CameraConfig xyStdDevCoeff(double v) {
        xyStdDevCoeff = v;
        return this;
    }

    public CameraConfig thetaStdDevCoeff(double v) {
        thetaStdDevCoeff = v;
        return this;
    }

    public CameraConfig maxTagDistance(double v) {
        maxTagDistance = v;
        return this;
    }

    public CameraConfig maxAmbiguity(double v) {
        maxAmbiguity = v;
        return this;
    }

    public CameraConfig maxMt1Mt2Disagreement(double v) {
        maxMt1Mt2Disagreement = v;
        return this;
    }

    public CameraConfig maxHeadingCorrectionDeg(double v) {
        maxHeadingCorrectionDeg = v;
        return this;
    }

    public CameraConfig headingResetStableSeconds(double v) {
        headingResetStableSeconds = v;
        return this;
    }

    public CameraConfig imuModes(int enabled, int disabled) {
        imuModeEnabled = enabled;
        imuModeDisabled = disabled;
        return this;
    }

    public CameraConfig throttleDisabled(int v) {
        throttleDisabled = v;
        return this;
    }

    /**
     * Sim mount from the Limelight's own robot-space numbers (web UI 3D tab, or pipeline json
     * rsf/rss/rsu/rsroll/rspitch/rsyaw). Limelight "side" is to the right and pitch is up, so both
     * flip sign into WPILib.
     */
    public CameraConfig simMount(
            double forward, double side, double up, double pitchDeg, double yawDeg) {
        simMount =
                new Transform3d(
                        forward,
                        -side,
                        up,
                        new Rotation3d(0, Math.toRadians(-pitchDeg), Math.toRadians(yawDeg)));
        return this;
    }

    public CameraConfig mount(Supplier<Pose3d> v) {
        mount = v;
        return this;
    }
}
