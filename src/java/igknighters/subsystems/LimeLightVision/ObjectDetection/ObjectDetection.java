package igknighters.subsystems.LimeLightVision.ObjectDetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.FieldVisualizer;
import igknighters.Robot;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightHelpers;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightHelpers.RawDetection;
import igknighters.subsystems.LimeLightVision.Localization.Limelight;
import igknighters.subsystems.LimeLightVision.ObjectDetection.ObjectDetectionSim.ObjectDetectionSim;
import igknighters.subsystems.LimeLightVision.ObjectDetection.ObjectTracker.Measurement;
import igknighters.subsystems.swerve.Swerve;
import igknighters.util.log.Log;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

/**
 * Turns Limelight neural detector results into field positions, by label.
 *
 * <p>Each detection's tx/ty is a ray out of the camera. It is cast from where the robot was when
 * the frame was captured onto the plane at the object's center height (objects are assumed to sit
 * on the floor).
 *
 * <p>Raw detections jump around (a low camera turns a little angle noise into a lot of range
 * noise), so they go through an {@link ObjectTracker} per label: what comes out is one steady
 * position per object, also when two cameras see it.
 */
public class ObjectDetection extends SubsystemBase {
    /** Sim only: an object on the field, sitting on the floor. */
    public record SimObject(String label, Translation2d position) {}

    private static final double DISCONNECT_SECONDS = 0.5;

    /** Floor on a detection's std dev (meters): box center vs object center, mount error. */
    static final double MIN_STD_DEV = 0.05;

    private static class Cam {
        final Limelight ll;
        final ObjectDetectionConfig cfg;
        final Alert disconnected;
        ObjectDetectionSim sim;
        double lastHeartbeat = -1;
        double lastHeartbeatChange = 0;

        Cam(ObjectDetectionConfig cfg) {
            this.cfg = cfg;
            this.ll = new Limelight(cfg.name);
            this.disconnected =
                    new Alert("Limelight " + cfg.name + " disconnected", AlertType.kError);
        }
    }

    private final Swerve swerve;
    private final List<Cam> cams = new ArrayList<>();
    private final Map<String, List<Translation2d>> objects = new HashMap<>();
    private final Map<String, ObjectTracker> trackers = new HashMap<>();
    private Boolean wasDisabled = null;

    /** Sim only: the objects on the field the simulated cameras can see. Edit freely. */
    public final List<SimObject> simObjects = new ArrayList<>();

    public ObjectDetection(Swerve swerve, List<ObjectDetectionConfig> configs) {
        this.swerve = swerve;
        for (ObjectDetectionConfig cfg : configs) {
            Cam cam = new Cam(cfg);
            if (Robot.isSimulation()) {
                cam.sim = new ObjectDetectionSim(cfg);
            }
            cams.add(cam);
            for (String label : cfg.labels) {
                objects.put(label, new ArrayList<>());
                trackers.put(label, new ObjectTracker());
            }
        }
    }

    /**
     * Field positions (wpiBlue, meters) of every object currently seen, one per object, keyed by
     * label. Every configured label is present, with an empty list when nothing sees it. An object
     * shows up after a few detections and drops out half a second after it was last seen.
     */
    public Map<String, List<Translation2d>> getObjects() {
        return objects;
    }

    /** {@link #getObjects()} for one label, empty when none are seen or the label is unknown. */
    public List<Translation2d> getObjects(String label) {
        return objects.getOrDefault(label, List.of());
    }

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();

        boolean disabled = DriverStation.isDisabled();
        if (wasDisabled == null || wasDisabled != disabled) { // throttle while disabled, stays cool
            wasDisabled = disabled;
            for (Cam cam : cams) {
                cam.ll.setThrottle(disabled ? cam.cfg.throttleDisabled : 0);
            }
        }

        List<Translation2d> raw = new ArrayList<>();
        for (Cam cam : cams) {
            double hb = cam.ll.getHeartbeat();
            boolean newFrame = hb != cam.lastHeartbeat;
            if (newFrame) {
                cam.lastHeartbeat = hb;
                cam.lastHeartbeatChange = now;
            }
            boolean connected = now - cam.lastHeartbeatChange < DISCONNECT_SECONDS;
            cam.disconnected.set(!connected);
            if (!connected || !newFrame) {
                // NetworkTables keeps the last values forever, do not trust a dead camera. And the
                // same frame fed twice would count as two sightings.
                continue;
            }
            // Same timestamp LimelightHelpers gives poses: when the frame arrived minus latency
            double captured =
                    LimelightHelpers.getLimelightNTTableEntry(cam.cfg.name, "hb").getLastChange()
                                    / 1e6
                            - (cam.ll.getPipelineLatencyMs() + cam.ll.getCaptureLatencyMs()) / 1e3;
            Map<String, List<Measurement>> frame = new HashMap<>();
            addDetections(cam.cfg, cam.ll.getRawDetections(), swerve.samplePoseAt(captured), frame);
            // every label of this camera, even unseen ones, then any class id missing from them
            for (String label : cam.cfg.labels) {
                frame.putIfAbsent(label, List.of());
            }
            for (var e : frame.entrySet()) {
                trackers.computeIfAbsent(e.getKey(), k -> new ObjectTracker())
                        .update(e.getValue(), now);
                for (Measurement m : e.getValue()) {
                    raw.add(m.position());
                }
            }
        }

        List<Pose2d> all = new ArrayList<>();
        for (var e : trackers.entrySet()) {
            List<Translation2d> list = objects.computeIfAbsent(e.getKey(), k -> new ArrayList<>());
            list.clear();
            e.getValue().confirmed(now, list);
            for (Translation2d p : list) {
                all.add(new Pose2d(p, Rotation2d.kZero));
            }
        }
        FieldVisualizer.getInstance().updateDetectedObjects(all);
        FieldVisualizer.getInstance().updateRawDetections(raw);
        if (!Robot.consts.limelightVision().disableVisionLogs()) {
            for (var e : objects.entrySet()) {
                Log.log(
                        "ObjectDetection/" + e.getKey(),
                        e.getValue().toArray(Translation2d[]::new));
            }
            Log.log("ObjectDetection/Raw", raw.toArray(Translation2d[]::new));
        }
    }

    @Override
    public void simulationPeriodic() {
        // by identity, so two objects stacked on the same spot are still two objects
        Set<SimObject> inView = Collections.newSetFromMap(new IdentityHashMap<>());
        for (Cam cam : cams) {
            // Truth, not our estimate, same as the localization sim
            inView.addAll(cam.sim.update(swerve.getSimTruthPose(), simObjects));
        }
        List<Translation2d> seen = new ArrayList<>();
        List<Translation2d> unseen = new ArrayList<>();
        for (SimObject obj : simObjects) {
            (inView.contains(obj) ? seen : unseen).add(obj.position());
        }
        FieldVisualizer.getInstance().updateSimObjects(seen, unseen);
        Log.log("Sim/ObjectsSeen", seen.toArray(Translation2d[]::new));
        Log.log("Sim/ObjectsUnseen", unseen.toArray(Translation2d[]::new));
    }

    /**
     * Put one frame's detections on the field. Class ids missing from the labels go under "class
     * &lt;id&gt;" so a wrong labels list shows up in the logs instead of vanishing.
     */
    static void addDetections(
            ObjectDetectionConfig cfg,
            RawDetection[] detections,
            Pose2d robotAtCapture,
            Map<String, List<Measurement>> out) {
        for (RawDetection d : detections) {
            String label =
                    d.classId >= 0 && d.classId < cfg.labels.length
                            ? cfg.labels[d.classId]
                            : "class " + d.classId;
            toField(robotAtCapture, cfg, d.txnc, d.tync)
                    .ifPresent(m -> out.computeIfAbsent(label, k -> new ArrayList<>()).add(m));
        }
    }

    /**
     * Where a detection at (tx, ty) lands on the plane {@code cfg.objectHeight} above the floor,
     * and how far off that may be. Limelight angles: tx positive right, ty positive up, each the
     * atan of that axis's normalized image coordinate, so the ray in camera space is (1, -tan tx,
     * tan ty). Empty when the ray never reaches the plane (at or above the horizon) or lands past
     * {@code maxObjectDistance}.
     *
     * <p>Range comes from the angle down to the object: with height gap h and range r, an angle
     * error d moves it (r^2 + h^2) / h * d. About 5 cm at 1 m but 40 cm at 3 m for a camera 12 cm
     * above the ball's center, which is why far detections must count for less.
     */
    static Optional<Measurement> toField(
            Pose2d robot, ObjectDetectionConfig cfg, double txDeg, double tyDeg) {
        Pose3d camera = new Pose3d(robot).plus(cfg.robotToCamera);
        Translation3d ray =
                new Translation3d(
                                1,
                                -Math.tan(Math.toRadians(txDeg)),
                                Math.tan(Math.toRadians(tyDeg)))
                        .rotateBy(camera.getRotation());
        double t = (cfg.objectHeight - camera.getZ()) / ray.getZ();
        if (!(t > 0)) { // also catches a level ray (t infinite or NaN)
            return Optional.empty();
        }
        Translation2d p = camera.getTranslation().plus(ray.times(t)).toTranslation2d();
        if (p.getDistance(robot.getTranslation()) > cfg.maxObjectDistance) {
            return Optional.empty();
        }
        double h = Math.abs(camera.getZ() - cfg.objectHeight);
        double r = p.getDistance(camera.getTranslation().toTranslation2d());
        double stdDev = (r * r + h * h) / h * Math.toRadians(cfg.angleNoiseDeg);
        return Optional.of(new Measurement(p, Math.max(MIN_STD_DEV, stdDev)));
    }
}
