package igknighters.subsystems.LimeLightVision;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Time;
import igknighters.Robot;
import igknighters.constants.FirstBotConsts;
import igknighters.subsystems.LimeLightVision.CameraData.Pipelines;
import igknighters.subsystems.LimeLightVision.Cameras.YallLimelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.Orientation3d;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class LimeLightVisionTest {

    private CameraData createData(String name, Pipelines pipeline) {
        return new CameraData(name, new Pose3d(new Translation3d(), new Rotation3d()), pipeline);
    }

    @BeforeEach
    void setOrientation() {
        // 1. Initialize static orientation to prevent NPE on periodic/limelight calls
        Robot.robotOrientation =
                new Orientation3d(
                        new Rotation3d(),
                        new AngularVelocity3d(RPM.of(0.0), RPM.of(0.0), RPM.of(0.0)));

        // 2. Guarantee Robot.consts is non-null for tests
        Robot.consts = new FirstBotConsts();
    }

    @Test
    void testCalculateScore() {
        YallLimelight camera = new YallLimelight(createData("testCam", Pipelines.POSE_DETECTION));

        // At ambiguity 0.0 and distance 4.0m: (2 - (0 + 1)) / 2 = 0.5
        double score = camera.calculateScore(0.0, 4.0);
        assertEquals(0.5, score, 1e-6, "Score at max range with 0 ambiguity should be 0.5");

        // Perfect conditions: ambiguity 0.0, distance 0.0m -> score 1.0
        assertEquals(1.0, camera.calculateScore(0.0, 0.0), 1e-6);
    }

    @Test
    void testUninitializedPipelineDoesNotThrowNPE() {
        CameraData data = createData("doesEverythingCam", Pipelines.DOES_EVERYTHING);
        YallLimelight camera = new YallLimelight(data);

        // Should handle null functioning_as_pipeline safely without throwing NullPointerException
        assertDoesNotThrow(
                () -> {
                    camera.getRobotPoseFromVision();
                });
    }

    @Test
    void testTimestampPersistenceWhenNoTargetsVisible() {
        LimeLightVision vision = new LimeLightVision();
        vision.latestMeasurementTime = Seconds.of(10.0);

        // Simulate query cycle where no vision measurements are returned
        vision.getRobotPoseFromVision();

        // Verify latestMeasurementTime was not reset to 0.0
        assertEquals(
                10.0,
                vision.getLastTimeStamp(),
                1e-6,
                "latestMeasurementTime should retain last valid timestamp when no targets are"
                        + " seen.");
    }

    @Test
    void testMicrosecondToMillisecondConversion() {
        long fpgaMicroseconds = 5_000_000L; // 5 seconds

        Time correctTime = Microseconds.of(fpgaMicroseconds);
        Time buggyTime = Milliseconds.of(fpgaMicroseconds);

        assertNotEquals(
                correctTime.in(Seconds),
                buggyTime.in(Seconds),
                "Microsecond FPGA timestamp mistakenly passed as Milliseconds causes 1000x scaling"
                        + " error.");
        assertEquals(5.0, correctTime.in(Seconds), 1e-6);
    }
}
