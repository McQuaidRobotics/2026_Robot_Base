package igknighters.subsystems.LimeLightVision.ObjectDetection;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightHelpers;
import igknighters.subsystems.LimeLightVision.ObjectDetection.ObjectDetection.SimObject;
import igknighters.subsystems.LimeLightVision.ObjectDetection.ObjectDetectionSim.ObjectDetectionSim;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class ObjectDetectionTest {
    @BeforeAll
    static void setup() {
        assertTrue(HAL.initialize(500, 0));
    }

    @Test
    void rayHitsTheFloorWhereGeometrySaysItShould() {
        // 1 m up, looking 45 deg down (Limelight pitch is up-positive)
        var cfg = new ObjectDetectionConfig("geom", 0, "fuel").mount(0, 0, 1, -45, 0);
        var robot = Pose2d.kZero;
        var center = ObjectDetection.toField(robot, cfg, 0, 0).orElseThrow();
        assertEquals(1, center.getX(), 1e-9);
        assertEquals(0, center.getY(), 1e-9);
        // tx positive = right of center = -y in WPILib
        assertTrue(ObjectDetection.toField(robot, cfg, 10, 0).orElseThrow().getY() < 0);
        // ty positive = up = farther away
        assertTrue(ObjectDetection.toField(robot, cfg, 0, 10).orElseThrow().getX() > 1);
        // level with or above the horizon never hits the floor
        assertTrue(ObjectDetection.toField(robot, cfg, 0, 45).isEmpty());
        assertTrue(ObjectDetection.toField(robot, cfg, 0, 60).isEmpty());
        // past maxObjectDistance is dropped
        assertTrue(ObjectDetection.toField(robot, cfg.maxObjectDistance(2), 0, 30).isEmpty());
    }

    @Test
    void simDetectionsLandBackWhereTheObjectsAre() {
        // the real limelight-back mount, looking out the back of the robot
        var cfg =
                new ObjectDetectionConfig("odtest", 0.075, "fuel", "robot")
                        .mount(-0.3129, -0.2237, 0.1994, 15, 180);
        var sim = new ObjectDetectionSim(cfg).perfect();
        var robot = new Pose2d(8, 4, Rotation2d.fromDegrees(30));
        var objects =
                List.of(
                        new SimObject("fuel", field(robot, -1.5, 0)),
                        new SimObject("robot", field(robot, -2.5, 0.6)),
                        new SimObject("fuel", field(robot, 2, 0)), // in front, back cam can't see
                        new SimObject("note", field(robot, -2, 0))); // not in the model

        sim.update(robot, objects);
        Map<String, List<Translation2d>> out = new HashMap<>();
        ObjectDetection.addDetections(cfg, LimelightHelpers.getRawDetections(cfg.name), robot, out);

        assertEquals(1, out.get("fuel").size());
        assertEquals(1, out.get("robot").size());
        assertEquals(2, out.size());
        assertEquals(0, out.get("fuel").get(0).getDistance(objects.get(0).position()), 0.01);
        assertEquals(0, out.get("robot").get(0).getDistance(objects.get(1).position()), 0.01);
        assertEquals("pipe_neuraldetector", LimelightHelpers.getCurrentPipelineType(cfg.name));
    }

    /** A robot-relative spot on the floor, in field coordinates. */
    private static Translation2d field(Pose2d robot, double x, double y) {
        return robot.getTranslation().plus(new Translation2d(x, y).rotateBy(robot.getRotation()));
    }
}
