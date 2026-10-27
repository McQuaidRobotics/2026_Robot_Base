package igknighters.subsystems.LimeLightVision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
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
import java.util.ArrayDeque;
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
    /**
     * One frame's correction. {@code headingReset}: MT1 has steadily said our heading is off by
     * {@code headingGap}; apply it in one step instead of as a filtered measurement.
     */
    public record Measurement(
            String camera,
            Pose2d pose,
            double timestamp,
            Matrix<N3, N1> stdDevs,
            boolean headingReset,
            Rotation2d headingGap) {}

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

        final HeadingGapTracker headingGap = new HeadingGapTracker();

        Cam(CameraConfig cfg) {
            this.cfg = cfg;
            this.ll = new Limelight(cfg.name);
            this.disconnected =
                    new Alert("Limelight " + cfg.name + " disconnected", AlertType.kError);
        }
    }

    /**
     * Decides whether MT1 has steadily disagreed with our heading by the same amount. A steady gap
     * means the gyro is a consistent amount out of sync with the camera (e.g. zeroed facing the
     * wrong way), not MT1 being noisy.
     *
     * <p>It is a vote over the last second, not "every frame in a row": one flipped, stale or
     * ambiguous frame used to restart the timer, and in sim (and likely on a real field) a run of
     * 50 clean frames almost never happened, so a wrong heading was never fixed. Untrusted frames
     * do not vote, flips get outvoted, and the current frame must be in the majority so a flip is
     * never the frame that gets applied. Package private so tests run the exact robot logic.
     */
    static class HeadingGapTracker {
        /** Fraction of trusted frames in the window that must agree with the current frame. */
        static final double AGREE_FRACTION = 0.8;

        /** Longer than this with no trusted frame and the vote starts over. */
        static final double MAX_SILENCE_SECONDS = 0.25;

        /** Trusted frames: {timestamp, gap degrees}. */
        private final ArrayDeque<double[]> window = new ArrayDeque<>();

        /** Start over, e.g. after the heading was reset and the old gap no longer applies. */
        void clear() {
            window.clear();
        }

        /** Most recent trusted MT1 frames say our heading is off by more than the nudge limit. */
        boolean headingSuspect(CameraConfig cfg) {
            int off = 0;
            for (double[] f : window) {
                if (Math.abs(f[1]) > cfg.maxHeadingCorrectionDeg) {
                    off++;
                }
            }
            return off * 2 > window.size();
        }

        /** Seconds the current gap has been the steady majority, 0 if it is not. */
        double update(CameraConfig cfg, PoseEstimate mt2, PoseEstimate mt1) {
            // the MT1 estimate is not trusted, see mt1Trusted for exclusion reasons
            if (!mt1Trusted(cfg, mt2, mt1, RESET_MIN_TAGS)) {
                return 0;
            }
            double t = mt1.timestampSeconds;
            double gap = headingGap(mt2, mt1).getDegrees();
            if (!window.isEmpty() && t - window.peekLast()[0] > MAX_SILENCE_SECONDS) {
                window.clear();
            }
            window.addLast(new double[] {t, gap});
            while (t - window.peekFirst()[0] > cfg.headingResetStableSeconds + 0.1) {
                window.removeFirst();
            }
            int agree = 0;
            for (double[] f : window) {
                if (Math.abs(MathUtil.inputModulus(f[1] - gap, -180, 180)) <= HEADING_STABLE_DEG) {
                    agree++;
                }
            }
            if (agree < AGREE_FRACTION * window.size()) {
                return 0;
            }
            return t - window.peekFirst()[0];
        }
    }

    /** Headings we sent the cameras, so a frame can be matched to the heading it was given. */
    private final TimeInterpolatableBuffer<Rotation2d> sentHeadings =
            TimeInterpolatableBuffer.createBuffer(1.5);

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
        sentHeadings.addSample(now, state.Pose.getRotation());
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
            alignMt2ToCaptureHeading(
                    mt2,
                    sentHeadings.getSample(mt2.timestampSeconds).orElse(state.Pose.getRotation()));
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
            Decision d = decide(cam.cfg, mt2, mt1, yawRateDps, cam.headingGap);
            String reason = d.reason();
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

            Measurement m = d.measurement();
            if (m.headingReset()) {
                // Shift by the gap, not to an absolute heading, so camera latency does not matter.
                // A half-fixed heading is what bends MT2, so it is fixed in one step.
                swerve.resetRotation(state.Pose.getRotation().plus(m.headingGap()));
                cam.headingGap.clear();
            } else {
                swerve.addVisionMeasurement(m.pose, m.timestamp, m.stdDevs);
            }
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

    /** What to do with one new frame: {@code reason} null means apply {@code measurement}. */
    record Decision(String reason, Measurement measurement) {}

    /**
     * Everything decided about one new frame, in one place so periodic and the tests run the same
     * logic. Call once per new frame per camera (the tracker remembers recent frames).
     */
    static Decision decide(
            CameraConfig cfg,
            PoseEstimate mt2,
            PoseEstimate mt1,
            double yawRateDps,
            HeadingGapTracker tracker) {
        double stable = tracker.update(cfg, mt2, mt1);
        String reason = rejectReason(cfg, mt2, mt1, yawRateDps, stable);
        // This frame's MT1 cannot vouch for our heading, but recent ones said it is off, so MT2
        // (built from our heading) is bent. Without this a 15 deg error dragged the pose 0.46 m.
        if (reason == null
                && !mt1Trusted(cfg, mt2, mt1, RESET_MIN_TAGS)
                && tracker.headingSuspect(cfg)) {
            reason = "heading suspect";
        }
        return new Decision(reason, reason == null ? measure(cfg, mt2, mt1) : null);
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
        if (mt2Unusable(cfg, mt2, mt1)) {
            // MegaTag2 is built on our heading, so a big disagreement means our heading is wrong
            // (or MegaTag2 failed, which reports field center). Only a trusted MT1 can fix heading,
            // and coplanar tags make MT1 flip between two headings, so it must hold one first.
            boolean resetOk =
                    mt1Trusted(cfg, mt2, mt1, RESET_MIN_TAGS)
                            && mt1StableSeconds >= cfg.headingResetStableSeconds
                            && !offField(mt1.pose);
            if (resetOk) return null;
            return mt2FailedToCenter(mt2) ? "MT2 failed (field center)" : "MT1/MT2 disagree";
        }
        if (offField(mt2.pose)) return "off field";
        return null;
    }

    /**
     * Normally: translation from MegaTag2, heading from MegaTag1 when trusted, otherwise heading is
     * left to the gyro (MegaTag2 heading is just our own gyro echoed back).
     *
     * <p>When MT2 is unusable (disagrees with MT1, failed, or MT1 says our heading is off) its
     * translation is garbage, so the result is a heading reset by the gap MT1 has been voting for.
     * Once the heading is fixed MT2 agrees again and translation flows.
     */
    static Measurement measure(CameraConfig cfg, PoseEstimate mt2, PoseEstimate mt1) {
        double theta = NO_TRUST;
        var rotation = mt2.pose.getRotation();
        boolean disagree = mt2Unusable(cfg, mt2, mt1); // distances dont match or MT2 failed
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
                    VecBuilder.fill(NO_TRUST, NO_TRUST, theta),
                    true,
                    headingGap(mt2, mt1));
        }
        double d = Math.max(mt2.avgTagDist, 0.5); // a 0 distance would mean infinite trust
        double xy = cfg.xyStdDevCoeff * d * d / mt2.tagCount;
        return new Measurement(
                cfg.name,
                new Pose2d(mt2.pose.getTranslation(), rotation),
                mt2.timestampSeconds,
                VecBuilder.fill(xy, xy, theta),
                false,
                Rotation2d.kZero);
    }

    /**
     * How far MT1 heading is from our heading in the same frame (MT2 heading is the heading we sent
     * for that frame). A wrong gyro keeps this constant even while the robot spins; MT1 flipping
     * makes it jump. So "stable" means stable gap, not stable heading, and works mid-match.
     */
    static Rotation2d headingGap(PoseEstimate mt2, PoseEstimate mt1) {
        return mt1.pose.getRotation().minus(mt2.pose.getRotation());
    }

    /**
     * MT1 and MT2 come from the same frame. On the robot their timestamps were identical (0.0 ms
     * apart), and one 120 fps frame is 8.3 ms, so 1 ms catches an MT1 left over from last frame.
     */
    static boolean sameFrame(PoseEstimate mt2, PoseEstimate mt1) {
        return mt1.tagCount > 0 && Math.abs(mt1.timestampSeconds - mt2.timestampSeconds) < 0.001;
    }

    /**
     * MT2 position cannot be used: it disagrees with MT1, it failed outright, or a trusted MT1 says
     * our heading is off by more than {@code maxHeadingCorrectionDeg}. MT2 builds its position from
     * our heading, so a 15 deg heading error bends it ~0.65 m around the tag at 2.5 m, which is
     * under the disagreement limit and used to drag the pose (caught by BadStartHeadingTest).
     */
    static boolean mt2Unusable(CameraConfig cfg, PoseEstimate mt2, PoseEstimate mt1) {
        return disagree(cfg, mt2, mt1)
                || mt2FailedToCenter(mt2)
                || (mt1Trusted(cfg, mt2, mt1, RESET_MIN_TAGS)
                        && Math.abs(headingGap(mt2, mt1).getDegrees())
                                > cfg.maxHeadingCorrectionDeg);
    }

    /**
     * When MegaTag2 cannot solve (usually our heading is badly wrong) the Limelight reports field
     * center (seen on the robot). Needed because near midfield the garbage is within the
     * disagreement threshold of the real pose and would otherwise be accepted. A robot really
     * within 5 cm of center only loses MT2 position for those frames.
     */
    static boolean mt2FailedToCenter(PoseEstimate mt2) {
        return Math.abs(mt2.pose.getX() - FieldConstants.X_FIELD / 2) < 0.05
                && Math.abs(mt2.pose.getY() - FieldConstants.Y_FIELD / 2) < 0.05;
    }

    /** Tag positions, only used as the pivot when straightening MT2. */
    static final AprilTagFieldLayout TAG_LAYOUT =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    /**
     * Make MT2 as if it had used our heading from when the image was captured.
     *
     * <p>MT2 solves with whatever heading the camera has when it processes the frame, which while
     * turning is newer than the image (130 deg/s x 50 ms = 6.5 deg in sim). That bends its position
     * around the tags, and worse, MT1 vs that heading showed a steady "gap" while spinning that the
     * vote would have "fixed" by breaking a good heading. So: rotate MT2's position back around the
     * tags it saw by (heading it used - heading at capture), and give it the capture heading. If
     * the camera already matched capture time, the difference is ~0 and nothing changes.
     *
     * <p>A failed MT2 (field center) reports heading 0 instead of the one it used; its position is
     * garbage anyway, so it only gets the capture heading (without that a 45 deg error was never
     * fixed, caught by BadStartHeadingTest).
     */
    static void alignMt2ToCaptureHeading(PoseEstimate mt2, Rotation2d headingAtCapture) {
        if (mt2.tagCount == 0) {
            return;
        }
        Translation2d position = mt2.pose.getTranslation();
        if (!mt2FailedToCenter(mt2)) {
            Rotation2d used = mt2.pose.getRotation();
            Translation2d pivot = Translation2d.kZero;
            int n = 0;
            for (var f : mt2.rawFiducials) {
                var tag = TAG_LAYOUT.getTagPose(f.id);
                if (tag.isPresent()) {
                    pivot = pivot.plus(tag.get().getTranslation().toTranslation2d());
                    n++;
                }
            }
            if (n > 0) {
                pivot = pivot.div(n);
                position = position.minus(pivot).rotateBy(headingAtCapture.minus(used)).plus(pivot);
            }
        }
        mt2.pose = new Pose2d(position, headingAtCapture);
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
