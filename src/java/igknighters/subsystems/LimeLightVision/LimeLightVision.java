package igknighters.subsystems.LimeLightVision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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

    private static class Cam {
        final Limelight ll;
        final CameraConfig cfg;
        final Alert disconnected;
        VisionSimulator sim;
        double lastFrameTimestamp = -1;
        double lastHeartbeat = -1;
        double lastHeartbeatChange = 0;

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

            String reason = rejectReason(cam.cfg, mt2, yawRateDps);
            if (log) {
                Log.log(path + "RejectReason", reason == null ? "accepted" : reason);
            }
            if (reason != null) {
                continue;
            }

            Measurement m = measure(cam.cfg, mt2, cam.ll.getMegaTag1());
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

    /** null if the MegaTag2 frame is usable, otherwise why it is not. */
    static String rejectReason(CameraConfig cfg, PoseEstimate mt2, double yawRateDps) {
        if (mt2.tagCount == 0) return "no tags";
        if (Math.abs(yawRateDps) > MAX_YAW_RATE_DPS) return "spinning too fast";
        if (mt2.avgTagDist > cfg.maxTagDistance) return "tags too far";
        Pose2d p = mt2.pose;
        if (p.getX() < -FIELD_MARGIN
                || p.getY() < -FIELD_MARGIN
                || p.getX() > FieldConstants.X_FIELD + FIELD_MARGIN
                || p.getY() > FieldConstants.Y_FIELD + FIELD_MARGIN) {
            return "off field";
        }
        return null;
    }

    /**
     * Translation from MegaTag2. Heading from MegaTag1 when it saw 2+ tags in the same frame,
     * otherwise heading is left to the gyro (MegaTag2 heading is just our own gyro echoed back).
     */
    static Measurement measure(CameraConfig cfg, PoseEstimate mt2, PoseEstimate mt1) {
        double d = Math.max(mt2.avgTagDist, 0.5); // a 0 distance would mean infinite trust
        double xy = cfg.xyStdDevCoeff * d * d / mt2.tagCount;

        boolean mt1Heading =
                mt1.tagCount >= 2 && Math.abs(mt1.timestampSeconds - mt2.timestampSeconds) < 0.01;
        double theta = NO_TRUST;
        var rotation = mt2.pose.getRotation();
        if (mt1Heading) {
            double d1 = Math.max(mt1.avgTagDist, 0.5);
            theta = cfg.thetaStdDevCoeff * d1 * d1 / mt1.tagCount;
            rotation = mt1.pose.getRotation();
        }
        return new Measurement(
                cfg.name,
                new Pose2d(mt2.pose.getTranslation(), rotation),
                mt2.timestampSeconds,
                VecBuilder.fill(xy, xy, theta));
    }
}
