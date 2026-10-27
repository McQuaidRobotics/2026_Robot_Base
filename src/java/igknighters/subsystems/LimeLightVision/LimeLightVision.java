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
import igknighters.subsystems.LimeLightVision.Sim.LimelightSim;
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

    /**
     * A single tag may reset heading because the reset also needs a 1 s stable gap (one tag held 9
     * s within 5 degrees on the robot). Per-frame nudges have no such check, so they need 2 tags.
     */
    private static final int RESET_MIN_TAGS = 1;

    private static final int NUDGE_MIN_TAGS = 2;

    private static class Cam {
        final Limelight ll;
        final CameraConfig cfg;
        final Alert disconnected;
        LimelightSim sim;
        double lastFrameTimestamp = -1;
        double lastHeartbeat = -1;
        double lastHeartbeatChange = 0;

        /** When MT1 started holding its current heading, -1 if it is not trusted right now. */
        double mt1StableSince = -1;

        Rotation2d mt1StableGap =
                Rotation2d.kZero; // this is if the gyro is a consistent amount out of sync with

        // camera which means that the camera is holding a consistent gap
        // with the gyro so something is wrong with gyro eg zeroed wrong

        Cam(CameraConfig cfg) {
            this.cfg = cfg;
            this.ll = new Limelight(cfg.name);
            this.disconnected =
                    new Alert("Limelight " + cfg.name + " disconnected", AlertType.kError);
        }
    }

    private final Swerve swerve;
    private final List<Cam> cams = new ArrayList<>();
    private final Map<String, Integer> cameraErrorCounts =
            new HashMap<>(); // this will help track if a camera is consistently off. This is
    // especially important because when the algorithim is bulletproofed
    // the bad measurements will be droped giving no indication of errors
    private final Map<String, Limelight> byName = new HashMap<>();
    private final List<Integer> visibleTagIds = new ArrayList<>();
    private final List<Measurement> accepted = new ArrayList<>();
    private final List<Integer> acceptedTagIds = new ArrayList<>();
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
                cams.get(i).sim = new LimelightSim(cams.get(i).cfg.name, cams.get(i).cfg.simMount);
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
    public List<Integer> getAllVisibleTagIds() {
        return visibleTagIds;
    }

    public List<Integer> getAcceptedTagIds() {
        return acceptedTagIds;
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

        // above is simply data gathering for the below math

        boolean disabled = DriverStation.isDisabled();
        if (wasDisabled == null
                || wasDisabled
                        != disabled) { // automatically enable and disable the vision system to not
            // overheat the cameras
            wasDisabled = disabled;
            for (Cam cam : cams) {
                cam.ll.setThrottle(disabled ? cam.cfg.throttleDisabled : 0);
                cam.ll.setIMUMode(disabled ? cam.cfg.imuModeDisabled : cam.cfg.imuModeEnabled);
            }
        }

        for (Cam cam : cams) {
            if (cam.cfg.mount
                    != null) { // if the camera has a moving position this will set the limelights
                // orientation in space
                // warning! From my expierience when you use this method it overwrites the values
                // held in gui.
                // I have not fully verified it but if you are doing any kind of testing with
                // turreted cameras please save the settings
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
        visibleTagIds.clear(); // dont make the list infinite and hold from previous runs
        accepted.clear();
        acceptedTagIds.clear();
        for (Cam cam : cams) {
            String path = "Vision/" + cam.cfg.name + "/";

            double hb = cam.ll.getHeartbeat();
            if (hb != cam.lastHeartbeat) { // the camera has processed a new frame since the last
                // robot loop
                cam.lastHeartbeat = hb;
                cam.lastHeartbeatChange = now;
            }
            boolean connected =
                    now - cam.lastHeartbeatChange
                            < DISCONNECT_SECONDS; // the camera is connected if it has sent a frame
            // recently
            cam.disconnected.set(!connected); // an alert that will notify ds that cam disconected
            if (!connected) {
                continue; // NetworkTables keeps the last values forever, do not trust them or else
                // it will keep sending pose from when turned off
            }

            List<Integer> tagsSeen = new ArrayList<>();

            PoseEstimate mt2 = cam.ll.getMegaTag2();
            for (var f : mt2.rawFiducials) {
                visibleTagIds.add(
                        f.id); // these are just what do i see no gating as to whether they are good
                // or not
                tagsSeen.add(f.id);
            }
            if (!cam.cfg.localize || mt2.timestampSeconds == cam.lastFrameTimestamp) {
                continue; // same frame as last loop, the estimator already has it
            }
            cam.lastFrameTimestamp = mt2.timestampSeconds;

            PoseEstimate mt1 =
                    cam.ll.getMegaTag1(); // we use this so we can fuse the two estimators and its
            // cheap to get because its no more math
            if (!mt1Trusted(
                    cam.cfg,
                    mt2,
                    mt1,
                    RESET_MIN_TAGS)) { // the MT1 estimate is not trusted see java docs for
                // exclusion reasons
                cam.mt1StableSince = -1; // -1 means not trusted
            } else if (cam.mt1StableSince < 0
                    || Math.abs(headingGap(mt2, mt1).minus(cam.mt1StableGap).getDegrees())
                            > HEADING_STABLE_DEG) { // the estimate is valid and the gap and
                // timestamp are set nothing will change unless
                // it has been stable for a while
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
            if (reason
                    != null) { // null means all good anything else means something wrong with frame
                // so no math gets done on it
                cameraErrorCounts.put(
                        cam.cfg.name, cameraErrorCounts.getOrDefault(cam.cfg.name, 0) + 1);
                continue;
            }

            Measurement m = measure(cam.cfg, mt2, mt1);
            swerve.addVisionMeasurement(m.pose, m.timestamp, m.stdDevs);
            accepted.add(m);
            acceptedTagIds.addAll(tagsSeen);
            if (log) {
                Log.log(path + "Pose", m.pose);
                Log.log(path + "StdDevs", m.stdDevs.getData());
                Log.log(path + "TagCount", mt2.tagCount);
            }
        }
        if (log) {
            Log.log("Vision/VisibleTags", visibleTagIds.stream().mapToLong(i -> i).toArray());
            Log.log("Vision/AcceptedCount", accepted.size());
            if (true) { // change this to false when you dont want to be in really deep debug i dont
                // know how big the lists will get and i envision it being expensive
                for (String key : cameraErrorCounts.keySet()) {
                    Log.log("Vision/" + key + "ErrorCount", cameraErrorCounts.get(key));
                }
            }
        }
    }

    @Override
    public void simulationPeriodic() {
        for (Cam cam : cams) {
            // Truth, not our estimate: feeding the estimate back in made the pose random walk
            cam.sim.update(swerve.getSimTruthPose());
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
                    mt1Trusted(cfg, mt2, mt1, RESET_MIN_TAGS)
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
        boolean disagree = disagree(cfg, mt2, mt1); // distances dont match or frames dont match
        double correction =
                mt1.pose
                        .getRotation()
                        .minus(mt2.pose.getRotation())
                        .getDegrees(); // error between two estimates
        // Agreeing frames may only nudge heading. Big changes go through the stable reset path.
        if (mt1Trusted(cfg, mt2, mt1, disagree ? RESET_MIN_TAGS : NUDGE_MIN_TAGS)
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

    /**
     * Enough tags, every one unambiguous. Ambiguity alone does not catch flips (seen on the robot),
     * which is why resets also need a stable heading gap and nudges are capped.
     */
    static boolean mt1Trusted(CameraConfig cfg, PoseEstimate mt2, PoseEstimate mt1, int minTags) {
        return sameFrame(mt2, mt1) // the timestamps match
                && mt1.tagCount >= minTags // enough tags to even consider mt1 unstable at like 1
                && mt1.rawFiducials.length == mt1.tagCount
                && maxAmbiguity(mt1) < cfg.maxAmbiguity; // every tag in the estimate is unambiguous
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
