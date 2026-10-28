package igknighters.subsystems.LimeLightVision.Localization;

import static igknighters.subsystems.LimeLightVision.Localization.LocalizationVision.NO_TRUST;
import static igknighters.subsystems.LimeLightVision.Localization.LocalizationVision.measure;
import static igknighters.subsystems.LimeLightVision.Localization.LocalizationVision.rejectReason;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import igknighters.subsystems.LimeLightVision.CameraConfig;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightHelpers.PoseEstimate;
import igknighters.subsystems.LimeLightVision.Localization.LocalizationSim.LimelightSim;
import igknighters.subsystems.LimeLightVision.Localization.LocalizationSim.LimelightSimSettings;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Sim faults go through NetworkTables and the real vision rules, like on the robot. */
class LimelightSimFaultTest {
    // Each test gets its own camera: NetworkTables keeps the old timestamp when a noiseless sim
    // republishes identical values, which would look like a stale frame.
    static int nextCam = 0;
    CameraConfig cfg = new CameraConfig("faulttest" + nextCam++);
    static Pose2d truth;

    @BeforeAll
    static void setup() {
        assertTrue(HAL.initialize(500, 0));
        // 2.5 m in front of the blue hub face (tags 19 and 20), facing it
        var layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        Pose2d tag19 = layout.getTagPose(19).orElseThrow().toPose2d();
        truth = tag19.transformBy(new Transform2d(2.5, 0.18, Rotation2d.k180deg));
    }

    /** One frame: tell the "camera" our heading, let the sim publish, read it back raw. */
    PoseEstimate[] frame(LimelightSim sim, Limelight ll, double ourHeadingDeg) {
        ll.setRobotOrientation(ourHeadingDeg, 0);
        sim.update(truth);
        return new PoseEstimate[] {ll.getMegaTag2(), ll.getMegaTag1()};
    }

    LimelightSim sim(LimelightSimSettings settings) {
        return new LimelightSim(cfg.name, settings); // camera at robot center, facing forward
    }

    @Test
    void wrongGyroMakesMt2ReportFieldCenterAndItIsRejected() {
        var ll = new Limelight(cfg.name);
        var sim = sim(LimelightSimSettings.perfect());
        var f = frame(sim, ll, truth.getRotation().getDegrees() + 90);
        assertTrue(f[0].tagCount > 0);
        assertEquals(8.27, f[0].pose.getX(), 0.05); // field center, like the real camera did
        assertEquals("MT2 failed (field center)", rejectReason(cfg, f[0], f[1], 0, 0));
        // after MT1 holds steady for a second it may fix heading, and only heading
        assertNull(rejectReason(cfg, f[0], f[1], 0, 1.0));
        var m = measure(cfg, f[0], f[1]);
        assertEquals(truth.getRotation().getDegrees(), m.pose().getRotation().getDegrees(), 1);
        assertEquals(NO_TRUST, m.stdDevs().get(0, 0));
    }

    @Test
    void smallGyroErrorSwingsMt2AroundTheTag() {
        var ll = new Limelight(cfg.name);
        var sim = sim(LimelightSimSettings.perfect());
        var f = frame(sim, ll, truth.getRotation().getDegrees() + 10);
        double off = f[0].pose.getTranslation().getDistance(truth.getTranslation());
        assertEquals(2.5 * Math.toRadians(10), off, 0.1); // ~0.44 m, like a real MT2
    }

    @Test
    void mt1FlipsNeverSwingHeading() {
        var ll = new Limelight(cfg.name);
        // always flipping, nothing else
        var sim = sim(LimelightSimSettings.perfect().withFaults(1, 0, 0, 0, 0));
        for (int i = 0; i < 20; i++) {
            var f = frame(sim, ll, truth.getRotation().getDegrees());
            String reason = rejectReason(cfg, f[0], f[1], 0, 0);
            if (reason == null) {
                assertEquals(NO_TRUST, measure(cfg, f[0], f[1]).stdDevs().get(2, 0));
            }
        }
    }

    @Test
    void staleMt1IsRejected() throws InterruptedException {
        var ll = new Limelight(cfg.name);
        var sim = sim(LimelightSimSettings.perfect());
        frame(sim, ll, truth.getRotation().getDegrees());
        Thread.sleep(100); // next frame comes later; MT1 keeps the old frame's timestamp
        // normal noise so MT2 is a genuinely new frame; only the stale-MT1 fault is on
        var stale = sim(new LimelightSimSettings().withFaults(0, 1, 0, 1, 0));
        var f = frame(stale, ll, truth.getRotation().getDegrees());
        System.out.println(
                "DEBUG mt2 ts "
                        + f[0].timestampSeconds
                        + " tags "
                        + f[0].tagCount
                        + " | mt1 ts "
                        + f[1].timestampSeconds
                        + " tags "
                        + f[1].tagCount);
        assertEquals("no MT1 in same frame", rejectReason(cfg, f[0], f[1], 0, 0));
    }
}
