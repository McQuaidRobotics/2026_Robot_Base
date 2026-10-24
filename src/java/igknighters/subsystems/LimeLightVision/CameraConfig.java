package igknighters.subsystems.LimeLightVision;

import edu.wpi.first.math.geometry.Pose3d;
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

    public double xyStdDevCoeff = 0.02;
    public double thetaStdDevCoeff = 0.06;

    /** Ignore frames whose average tag distance is farther than this (meters). */
    public double maxTagDistance = 5.0;

    /** MegaTag1 heading is only trusted when every tag's ambiguity is below this. */
    public double maxAmbiguity = 0.3;

    /** MT1 vs MT2 translation gap (meters) that means our heading is wrong. */
    public double maxMt1Mt2Disagreement = 1.0;

    /** Most an agreeing frame's MegaTag1 heading may move the robot heading (degrees). */
    public double maxHeadingCorrectionDeg = 30;

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

    public CameraConfig mount(Supplier<Pose3d> v) {
        mount = v;
        return this;
    }
}
