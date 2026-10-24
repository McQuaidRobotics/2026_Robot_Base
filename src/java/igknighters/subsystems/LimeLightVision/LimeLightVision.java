package igknighters.subsystems.LimeLightVision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.Robot;
import igknighters.constants.FieldConstants;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightHelpers;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightHelpers.PoseEstimate;
import igknighters.subsystems.LimeLightVision.Helpers.VisionSimulator;
import igknighters.subsystems.swerve.Swerve;
import igknighters.util.log.Log;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Turns raw Limelight data into drivetrain pose corrections.
 *
 * <p>Every new frame from every camera is judged on its own (accept or reject, then std devs) and
 * handed to the swerve pose estimator with that frame's own timestamp. The estimator's Kalman
 * filter does the fusing, so nothing is averaged here.
 *
 * <p>Other code reads cameras through {@link #camera(String)}, which exposes every Limelight
 * feature raw. Adding object detection, tag tracking, or a turret camera is a {@link CameraConfig}
 * plus a consumer of {@link Limelight}; this file does not change.
 */
public class LimeLightVision extends SubsystemBase {
    public record Measurement(
            String camera, Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {}

    /** MegaTag2 is unreliable above this spin rate (Limelight docs). */
    static final double MAX_YAW_RATE_DPS = 720;

    static final double FIELD_MARGIN = 0.5;

    /** Std dev for an axis we do not want corrected at all. */
    static final double NO_TRUST = 1e6;

    private static final double DISCONNECT_SECONDS = 0.5;
    private static final double HEADING_STABLE_DEG = 5;

    private static class Cam {
        final Limelight ll;
        final CameraConfig cfg;
        final Alert disconnected;
        VisionSimulator sim;
        double lastFrameTimestamp = -1;
        double lastHeartbeat = -1;
        double lastHeartbeatChange = 0;

        /** When MT1 started holding its current heading, -1 if it is not trusted right now. */
        double mt1StableSince = -1;

        Rotation2d mt1StableGap = Rotation2d.kZero;

        Cam(CameraConfig cfg) {
            this.cfg = cfg;
            this.ll = new Limelight(cfg.name);
            this.disconnected =
                    new Alert("Limelight " + cfg.name + " disconnected", AlertType.kError);
        }
    }

    private final Swerve swerve;
    private final List<Cam> cams = new ArrayList<>();
    private final Map<String, Limelight> byName = new HashMap<>();
    private final List<Integer> visibleTagIds = new ArrayList<>();
    private final List<Measurement> accepted = new ArrayList<>();
    private Boolean wasDisabled = null;

    public LimeLightVision(Swerve swerve, List<CameraConfig> configs) {
        this.swerve = swerve;
        for (CameraConfig cfg : configs) {
            Cam cam = new Cam(cfg);
            cams.add(cam);
            byName.put(cfg.name, cam.ll);
        }
        if (Robot.isSimulation()) {
            for (int i = 0; i < cams.size(); i++) {
                cams.get(i).sim = VisionSimulator.forCameraIndex(i);
            }
        }
    }

    /**
     * Raw access to one camera, e.g. {@code vision.camera("limelight-intake").getRawDetections()}.
     */
    public Limelight camera(String name) {
        Limelight ll = byName.get(name);
        if (ll == null) {
            throw new IllegalArgumentException("No Limelight configured named " + name);
        }
        return ll;
    }

    /** Tag ids seen by any connected camera in its latest frame. */
    public List<Integer> getVisibleTagIds() {
        return visibleTagIds;
    }

    /** Measurements sent to the drivetrain this loop. */
    public List<Measurement> getAcceptedMeasurements() {
        return accepted;
    }

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        var state = swerve.getState();
        double yawDeg = state.Pose.getRotation().getDegrees();
        double yawRateDps = Math.toDegrees(state.Speeds.omegaRadiansPerSecond);

        boolean disabled = DriverStation.isDisabled();
        if (wasDisabled == null || wasDisabled != disabled) {
            wasDisabled = disabled;
            for (Cam cam : cams) {
                cam.ll.setThrottle(disabled ? cam.cfg.throttleDisabled : 0);
                cam.ll.setIMUMode(disabled ? cam.cfg.imuModeDisabled : cam.cfg.imuModeEnabled);
            }
        }

        for (Cam cam : cams) {
            if (cam.cfg.mount != null) {
                Pose3d m = cam.cfg.mount.get();
                cam.ll.setCameraPoseRobotSpace(
                        m.getX(),
                        m.getY(),
                        m.getZ(),
                        Math.toDegrees(m.getRotation().getX()),
                        Math.toDegrees(m.getRotation().getY()),
                        Math.toDegrees(m.getRotation().getZ()));
            }
            cam.ll.setRobotOrientation(yawDeg, yawRateDps);
        }
        LimelightHelpers.Flush();

        boolean log = !Robot.consts.limelightVision().disableVisionLogs();
        visibleTagIds.clear();
        accepted.clear();
        for (Cam cam : cams) {
            String path = "Vision/" + cam.cfg.name + "/";

            double hb = cam.ll.getHeartbeat();
            if (hb != cam.lastHeartbeat) {
                cam.lastHeartbeat = hb;
                cam.lastHeartbeatChange = now;
            }
            boolean connected = now - cam.lastHeartbeatChange < DISCONNECT_SECONDS;
            cam.disconnected.set(!connected);
            if (!connected) {
                continue; // NetworkTables keeps the last values forever, do not trust them
            }

            PoseEstimate mt2 = cam.ll.getMegaTag2();
            for (var f : mt2.rawFiducials) {
                visibleTagIds.add(f.id);
            }
            if (!cam.cfg.localize || mt2.timestampSeconds == cam.lastFrameTimestamp) {
                continue; // same frame as last loop, the estimator already has it
            }
            cam.lastFrameTimestamp = mt2.timestampSeconds;

            PoseEstimate mt1 = cam.ll.getMegaTag1();
            if (!mt1Trusted(cam.cfg, mt2, mt1)) {
                cam.mt1StableSince = -1;
            } else if (cam.mt1StableSince < 0
                    || Math.abs(headingGap(mt2, mt1).minus(cam.mt1StableGap).getDegrees())
                            > HEADING_STABLE_DEG) {
                cam.mt1StableSince = mt1.timestampSeconds;
                cam.mt1StableGap = headingGap(mt2, mt1);
            }
            double mt1StableSeconds =
                    cam.mt1StableSince < 0 ? 0 : mt1.timestampSeconds - cam.mt1StableSince;
            String reason = rejectReason(cam.cfg, mt2, mt1, yawRateDps, mt1StableSeconds);
            if (log) {
                Log.log(path + "RejectReason", reason == null ? "accepted" : reason);
                Log.log(path + "MT1", mt1.pose);
                Log.log(path + "MT2", mt2.pose);
                Log.log(path + "MaxAmbiguity", maxAmbiguity(mt1));
            }
            if (reason != null) {
                continue;
            }

            Measurement m = measure(cam.cfg, mt2, mt1);
            swerve.addVisionMeasurement(m.pose, m.timestamp, m.stdDevs);
            accepted.add(m);
            if (log) {
                Log.log(path + "Pose", m.pose);
                Log.log(path + "StdDevs", m.stdDevs.getData());
                Log.log(path + "TagCount", mt2.tagCount);
            }
        }
        if (log) {
            Log.log("Vision/VisibleTags", visibleTagIds.stream().mapToLong(i -> i).toArray());
            Log.log("Vision/AcceptedCount", accepted.size());
        }
    }

    @Override
    public void simulationPeriodic() {
        for (Cam cam : cams) {
            cam.sim.publish(cam.cfg.name);
        }
    }

    /** null if the frame is usable, otherwise why it is not. */
    static String rejectReason(
            CameraConfig cfg,
            PoseEstimate mt2,
            PoseEstimate mt1,
            double yawRateDps,
            double mt1StableSeconds) {
        if (mt2.tagCount == 0) return "no tags";
        if (Math.abs(yawRateDps) > MAX_YAW_RATE_DPS) return "spinning too fast";
        if (mt2.avgTagDist > cfg.maxTagDistance) return "tags too far";
        // MT1 sees the same tags, so no MT1 means nothing to cross-check MT2 against
        if (!sameFrame(mt2, mt1)) return "no MT1 in same frame";
        if (disagree(cfg, mt2, mt1)) {
            // MegaTag2 is built on our heading, so a big disagreement means our heading is wrong
            // (or MegaTag2 failed, which reports field center). Only a trusted MT1 can fix heading,
            // and coplanar tags make MT1 flip between two headings, so it must hold one first.
            boolean resetOk =
                    mt1Trusted(cfg, mt2, mt1)
                            && mt1StableSeconds >= cfg.headingResetStableSeconds
                            && !offField(mt1.pose);
            return resetOk ? null : "MT1/MT2 disagree";
        }
        if (offField(mt2.pose)) return "off field";
        return null;
    }

    /**
     * Normally: translation from MegaTag2, heading from MegaTag1 when trusted, otherwise heading is
     * left to the gyro (MegaTag2 heading is just our own gyro echoed back).
     *
     * <p>When MT1 and MT2 disagree, MT2 translation is garbage, so only MT1 heading is sent. Once
     * the heading is fixed MT2 agrees again and translation flows.
     */
    static Measurement measure(CameraConfig cfg, PoseEstimate mt2, PoseEstimate mt1) {
        double theta = NO_TRUST;
        var rotation = mt2.pose.getRotation();
        boolean disagree = disagree(cfg, mt2, mt1);
        double correction = mt1.pose.getRotation().minus(mt2.pose.getRotation()).getDegrees();
        // Agreeing frames may only nudge heading. Big changes go through the stable reset path.
        if (mt1Trusted(cfg, mt2, mt1)
                && (disagree || Math.abs(correction) <= cfg.maxHeadingCorrectionDeg)) {
            double d1 = Math.max(mt1.avgTagDist, 0.5);
            theta = cfg.thetaStdDevCoeff * d1 * d1 / mt1.tagCount;
            rotation = mt1.pose.getRotation();
        }
        if (disagree) {
            return new Measurement(
                    cfg.name,
                    new Pose2d(mt1.pose.getTranslation(), rotation),
                    mt2.timestampSeconds,
                    VecBuilder.fill(NO_TRUST, NO_TRUST, theta));
        }
        double d = Math.max(mt2.avgTagDist, 0.5); // a 0 distance would mean infinite trust
        double xy = cfg.xyStdDevCoeff * d * d / mt2.tagCount;
        return new Measurement(
                cfg.name,
                new Pose2d(mt2.pose.getTranslation(), rotation),
                mt2.timestampSeconds,
                VecBuilder.fill(xy, xy, theta));
    }

    /**
     * How far MT1 heading is from our heading in the same frame (MT2 heading is the heading we sent
     * for that frame). A wrong gyro keeps this constant even while the robot spins; MT1 flipping
     * makes it jump. So "stable" means stable gap, not stable heading, and works mid-match.
     */
    static Rotation2d headingGap(PoseEstimate mt2, PoseEstimate mt1) {
        return mt1.pose.getRotation().minus(mt2.pose.getRotation());
    }

    static boolean sameFrame(PoseEstimate mt2, PoseEstimate mt1) {
        return mt1.tagCount > 0 && Math.abs(mt1.timestampSeconds - mt2.timestampSeconds) < 0.01;
    }

    static boolean disagree(CameraConfig cfg, PoseEstimate mt2, PoseEstimate mt1) {
        return sameFrame(mt2, mt1)
                && mt1.pose.getTranslation().getDistance(mt2.pose.getTranslation())
                        > cfg.maxMt1Mt2Disagreement;
    }

    /** 2+ tags, every one unambiguous. Coplanar tags can still flip between two solutions. */
    static boolean mt1Trusted(CameraConfig cfg, PoseEstimate mt2, PoseEstimate mt1) {
        return sameFrame(mt2, mt1)
                && mt1.tagCount >= 2
                && mt1.rawFiducials.length == mt1.tagCount
                && maxAmbiguity(mt1) < cfg.maxAmbiguity;
    }

    static double maxAmbiguity(PoseEstimate est) {
        double max = 0;
        for (var f : est.rawFiducials) {
            max = Math.max(max, f.ambiguity);
        }
        return max;
    }

    static boolean offField(Pose2d p) {
        return p.getX() < -FIELD_MARGIN
                || p.getY() < -FIELD_MARGIN
                || p.getX() > FieldConstants.X_FIELD + FIELD_MARGIN
                || p.getY() > FieldConstants.Y_FIELD + FIELD_MARGIN;
    }
}
