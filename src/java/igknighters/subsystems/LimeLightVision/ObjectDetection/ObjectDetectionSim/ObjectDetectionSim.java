package igknighters.subsystems.LimeLightVision.ObjectDetection.ObjectDetectionSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import igknighters.subsystems.LimeLightVision.ObjectDetection.ObjectDetection.SimObject;
import igknighters.subsystems.LimeLightVision.ObjectDetection.ObjectDetectionConfig;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.Random;

/**
 * Simulates one Limelight on a neural detector pipeline by publishing the keys a real one does
 * (rawdetections, tv/tx/ty/txnc/tync/ta, tdclass, t2d, tl/cl, hb, getpipetype), so robot code runs
 * unchanged in sim. YALL only simulates AprilTags, so this follows the same design as
 * LocalizationSim's LimelightSim: fed the TRUE robot pose every loop, pinhole camera, Gaussian
 * noise on the angles.
 *
 * <p>Not simulated: occlusion (objects behind other objects or the robot are still seen), false
 * positives and missed detections. Objects are spheres of diameter 2 * objectHeight for box size.
 */
public class ObjectDetectionSim {
    // Defaults approximate a Limelight 3 at 1280x960, same as the AprilTag sim. Public so tests and
    // tuning can change them.
    public double resolutionWidth = 1280;
    public double resolutionHeight = 960;
    public double horizontalFovDeg = 63.3;
    public double verticalFovDeg = 49.7;
    public double maxRangeMeters = 6.0;
    public double angleNoiseStdDevDeg = 0.3;
    public double avgPipelineLatencyMs = 30;
    public double latencyStdDevMs = 5;
    public double avgCaptureLatencyMs = 10;

    private final ObjectDetectionConfig cfg;
    private final NetworkTable table;
    private final Random noise;
    private final TimeInterpolatableBuffer<Pose2d> truthHistory =
            TimeInterpolatableBuffer.createBuffer(1.0);
    private long heartbeat = 0;

    private record Seen(int classId, String label, double tx, double ty, double[] box) {
        /** box is {minX, minY, maxX, maxY} in pixels. */
        double area() {
            return (box[2] - box[0]) * (box[3] - box[1]);
        }
    }

    public ObjectDetectionSim(ObjectDetectionConfig cfg) {
        this.cfg = cfg;
        this.table = NetworkTableInstance.getDefault().getTable(cfg.name);
        // seeded per camera so two cameras never publish identical noise
        this.noise = new Random(cfg.name.hashCode());
    }

    /** Turn off noise and latency, for deterministic tests. */
    public ObjectDetectionSim perfect() {
        angleNoiseStdDevDeg = 0;
        avgPipelineLatencyMs = 0;
        latencyStdDevMs = 0;
        avgCaptureLatencyMs = 0;
        return this;
    }

    /**
     * Publish one frame. Call every loop with the TRUE robot pose. Returns the objects in this
     * frame, for visualizing.
     */
    public List<SimObject> update(Pose2d robotPose, List<SimObject> objects) {
        double now = Timer.getFPGATimestamp();
        truthHistory.addSample(now, robotPose);
        double tlMs = Math.max(0, avgPipelineLatencyMs + noise.nextGaussian() * latencyStdDevMs);
        // Limelight timestamps a frame as (publish time - latency), so show where we were then
        Pose2d atCapture =
                truthHistory
                        .getSample(now - (tlMs + avgCaptureLatencyMs) / 1000.0)
                        .orElse(robotPose);
        Pose3d camera = new Pose3d(atCapture).plus(cfg.robotToCamera);

        List<Seen> seen = new ArrayList<>();
        List<SimObject> inView = new ArrayList<>();
        for (SimObject obj : objects) {
            int classId = Arrays.asList(cfg.labels).indexOf(obj.label());
            if (classId < 0) {
                continue; // the model was not trained on it
            }
            Translation3d center =
                    new Translation3d(
                            obj.position().getX(), obj.position().getY(), cfg.objectHeight);
            Translation3d c =
                    new Pose3d(center, Rotation3d.kZero).relativeTo(camera).getTranslation();
            project(classId, obj.label(), c)
                    .ifPresent(
                            s -> {
                                seen.add(s);
                                inView.add(obj);
                            });
        }
        seen.sort((a, b) -> Double.compare(b.area(), a.area()));
        publish(seen, tlMs);
        return inView;
    }

    /** One object at {@code c} (camera space, WPILib axes) into the image, if it is in view. */
    private Optional<Seen> project(int classId, String label, Translation3d c) {
        if (c.getX() <= 0.05 || c.getNorm() > maxRangeMeters) {
            return Optional.empty();
        }
        // Limelight convention: tx positive right, ty positive up, atan per image axis
        double tanX = -c.getY() / c.getX();
        double tanY = c.getZ() / c.getX();
        double fx = (resolutionWidth / 2) / Math.tan(Math.toRadians(horizontalFovDeg / 2));
        double fy = (resolutionHeight / 2) / Math.tan(Math.toRadians(verticalFovDeg / 2));
        double u = resolutionWidth / 2 + fx * tanX;
        double v = resolutionHeight / 2 - fy * tanY; // pixel y grows down
        if (u < 0 || u > resolutionWidth || v < 0 || v > resolutionHeight) {
            return Optional.empty();
        }
        double half = fx * cfg.objectHeight / c.getX(); // radius in pixels
        double[] box = {
            Math.max(0, u - half),
            Math.max(0, v - half),
            Math.min(resolutionWidth, u + half),
            Math.min(resolutionHeight, v + half)
        };
        double tx = Math.toDegrees(Math.atan(tanX)) + noise.nextGaussian() * angleNoiseStdDevDeg;
        double ty = Math.toDegrees(Math.atan(tanY)) + noise.nextGaussian() * angleNoiseStdDevDeg;
        return Optional.of(new Seen(classId, label, tx, ty, box));
    }

    private void publish(List<Seen> seen, double tlMs) {
        double imageArea = resolutionWidth * resolutionHeight;
        double[] raw = new double[12 * seen.size()];
        for (int i = 0; i < seen.size(); i++) {
            Seen s = seen.get(i);
            double[] b = s.box();
            System.arraycopy(
                    new double[] {
                        s.classId(),
                        s.tx(),
                        s.ty(),
                        s.area() / imageArea * 100,
                        b[0],
                        b[1],
                        b[2],
                        b[1],
                        b[2],
                        b[3],
                        b[0],
                        b[3]
                    },
                    0,
                    raw,
                    12 * i,
                    12);
        }
        Seen primary = seen.isEmpty() ? null : seen.get(0);
        double tx = primary == null ? 0 : primary.tx();
        double ty = primary == null ? 0 : primary.ty();
        double ta = primary == null ? 0 : primary.area() / imageArea * 100;

        table.getEntry("rawdetections").setDoubleArray(raw);
        table.getEntry("tv").setDouble(primary == null ? 0 : 1);
        // crosshair at the image center, so tx == txnc
        table.getEntry("tx").setDouble(tx);
        table.getEntry("ty").setDouble(ty);
        table.getEntry("txnc").setDouble(tx);
        table.getEntry("tync").setDouble(ty);
        table.getEntry("ta").setDouble(ta);
        table.getEntry("tdclass").setString(primary == null ? "" : primary.label());
        table.getEntry("t2d")
                .setDoubleArray(
                        new double[] {
                            primary == null ? 0 : 1,
                            seen.size(),
                            tlMs + avgCaptureLatencyMs,
                            0,
                            tx,
                            ty,
                            tx,
                            ty,
                            ta,
                            -1,
                            primary == null ? -1 : primary.classId(),
                            -1,
                            0,
                            0,
                            0,
                            0,
                            0
                        });
        table.getEntry("tl").setDouble(tlMs);
        table.getEntry("cl").setDouble(avgCaptureLatencyMs);
        table.getEntry("getpipetype").setString("pipe_neuraldetector");
        table.getEntry("hb").setDouble(heartbeat++);
    }
}
