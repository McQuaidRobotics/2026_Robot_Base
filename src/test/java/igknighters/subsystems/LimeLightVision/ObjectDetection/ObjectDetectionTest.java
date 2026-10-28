package igknighters.subsystems.LimeLightVision.ObjectDetection;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightHelpers;
import igknighters.subsystems.LimeLightVision.ObjectDetection.ObjectDetection.SimObject;
import igknighters.subsystems.LimeLightVision.ObjectDetection.ObjectDetectionSim.ObjectDetectionSim;
import igknighters.subsystems.LimeLightVision.ObjectDetection.ObjectTracker.Measurement;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class ObjectDetectionTest {
    static final Pose2d ROBOT = new Pose2d(8, 4, Rotation2d.fromDegrees(30));
    static int nextCam = 0;

    @BeforeAll
    static void setup() {
        assertTrue(HAL.initialize(500, 0));
    }

    /** The real limelight-back mount (looking out the back), own NT table per test. */
    static ObjectDetectionConfig backCam(String... labels) {
        return new ObjectDetectionConfig("odtest" + nextCam++, 0.075, labels)
                .mount(-0.3129, -0.2237, 0.1994, 15, 180);
    }

    /** A robot-relative spot on the floor, in field coordinates. */
    static Translation2d field(double x, double y) {
        return ROBOT.getTranslation().plus(new Translation2d(x, y).rotateBy(ROBOT.getRotation()));
    }

    /** One sim frame through NetworkTables and onto the field, like periodic does. */
    static Map<String, List<Measurement>> frame(
            ObjectDetectionConfig cfg, ObjectDetectionSim sim, List<SimObject> objects) {
        sim.update(ROBOT, objects);
        Map<String, List<Measurement>> out = new HashMap<>();
        ObjectDetection.addDetections(cfg, LimelightHelpers.getRawDetections(cfg.name), ROBOT, out);
        return out;
    }

    @Test
    void rayHitsTheFloorWhereGeometrySaysItShould() {
        // 1 m up, looking 45 deg down (Limelight pitch is up-positive)
        var cfg = new ObjectDetectionConfig("geom", 0, "fuel").mount(0, 0, 1, -45, 0);
        var robot = Pose2d.kZero;
        var center = ObjectDetection.toField(robot, cfg, 0, 0).orElseThrow().position();
        assertEquals(1, center.getX(), 1e-9);
        assertEquals(0, center.getY(), 1e-9);
        // tx positive = right of center = -y in WPILib
        assertTrue(ObjectDetection.toField(robot, cfg, 10, 0).orElseThrow().position().getY() < 0);
        // ty positive = up = farther away
        assertTrue(ObjectDetection.toField(robot, cfg, 0, 10).orElseThrow().position().getX() > 1);
        // level with or above the horizon never hits the floor
        assertTrue(ObjectDetection.toField(robot, cfg, 0, 45).isEmpty());
        assertTrue(ObjectDetection.toField(robot, cfg, 0, 60).isEmpty());
        // past maxObjectDistance is dropped
        assertTrue(ObjectDetection.toField(robot, cfg.maxObjectDistance(2), 0, 30).isEmpty());
    }

    @Test
    void farDetectionsAreTrustedLess() {
        var cfg = backCam("fuel");
        double near = ObjectDetection.toField(ROBOT, cfg, 0, -22).orElseThrow().stdDev();
        double far = ObjectDetection.toField(ROBOT, cfg, 0, -17.5).orElseThrow().stdDev();
        assertTrue(far > 3 * near, near + " vs " + far);
    }

    @Test
    void simDetectionsLandBackWhereTheObjectsAre() {
        var cfg = backCam("fuel", "robot");
        var sim = new ObjectDetectionSim(cfg).perfect();
        var objects =
                List.of(
                        new SimObject("fuel", field(-1.5, 0)),
                        new SimObject("robot", field(-2.5, 0.6)),
                        new SimObject("fuel", field(2, 0)), // in front, back cam can't see
                        new SimObject("note", field(-2, 0))); // not in the model

        var out = frame(cfg, sim, objects);

        assertEquals(1, out.get("fuel").size());
        assertEquals(1, out.get("robot").size());
        assertEquals(2, out.size());
        assertEquals(
                0, out.get("fuel").get(0).position().getDistance(objects.get(0).position()), 0.01);
        assertEquals(
                0, out.get("robot").get(0).position().getDistance(objects.get(1).position()), 0.01);
        assertEquals("pipe_neuraldetector", LimelightHelpers.getCurrentPipelineType(cfg.name));
    }

    /**
     * The flicker complaint: noisy sim, a ball sitting still. Once it shows up it must stay, stay
     * one object, and barely move.
     */
    void stillBallIsSteady(double distance, double maxError, double maxStep) {
        var cfg = backCam("fuel");
        var sim = new ObjectDetectionSim(cfg); // default noise
        var tracker = new ObjectTracker();
        var ball = field(-distance, 0);
        Translation2d last = null;
        double worstError = 0, worstStep = 0;
        for (int i = 0; i < 300; i++) {
            double now = i * 0.02;
            tracker.update(
                    frame(cfg, sim, List.of(new SimObject("fuel", ball)))
                            .getOrDefault("fuel", List.of()),
                    now);
            List<Translation2d> out = new ArrayList<>();
            tracker.confirmed(now, out);
            if (i < 25) {
                continue; // settling
            }
            assertEquals(1, out.size(), "frame " + i);
            worstError = Math.max(worstError, out.get(0).getDistance(ball));
            if (last != null) {
                worstStep = Math.max(worstStep, out.get(0).getDistance(last));
            }
            last = out.get(0);
        }
        assertTrue(worstError < maxError, distance + " m: worst error " + worstError);
        assertTrue(worstStep < maxStep, distance + " m: worst step " + worstStep);
    }

    @Test
    void stillBallIsSteadyAt2m() {
        stillBallIsSteady(2, 0.12, 0.075); // under a ball radius per frame
    }

    @Test
    void stillBallIsSteadyAt4m() {
        stillBallIsSteady(4, 0.5, 0.075); // 12 cm camera height can not range better at 4 m
    }

    @Test
    void closeBallsStayTwoObjects() {
        var cfg = backCam("fuel");
        var sim = new ObjectDetectionSim(cfg);
        var tracker = new ObjectTracker();
        var objects =
                List.of(
                        new SimObject("fuel", field(-1.5, 0.2)),
                        new SimObject("fuel", field(-1.5, -0.2)));
        for (int i = 0; i < 150; i++) {
            double now = i * 0.02;
            tracker.update(frame(cfg, sim, objects).getOrDefault("fuel", List.of()), now);
            List<Translation2d> out = new ArrayList<>();
            tracker.confirmed(now, out);
            if (i >= 25) {
                assertEquals(2, out.size(), "frame " + i);
            }
        }
    }

    @Test
    void removedBallIsGoneWithinMaxAge() {
        var tracker = new ObjectTracker();
        var m = new Measurement(new Translation2d(3, 3), 0.05);
        double now = 0;
        for (int i = 0; i < 20; i++, now += 0.02) {
            tracker.update(List.of(m), now);
        }
        List<Translation2d> out = new ArrayList<>();
        tracker.confirmed(now, out);
        assertEquals(1, out.size());
        double pickedUp = now;
        for (; now < pickedUp + ObjectTracker.MAX_AGE_SECONDS + 0.05; now += 0.02) {
            tracker.update(List.of(), now);
        }
        out.clear();
        tracker.confirmed(now, out);
        assertEquals(0, out.size());
    }

    @Test
    void oneFrameStrayNeverShowsUp() {
        var tracker = new ObjectTracker();
        tracker.update(List.of(new Measurement(new Translation2d(5, 5), 0.05)), 0);
        for (double now = 0.02; now < 1; now += 0.02) {
            tracker.update(List.of(), now);
            List<Translation2d> out = new ArrayList<>();
            tracker.confirmed(now, out);
            assertEquals(0, out.size());
        }
    }
}
