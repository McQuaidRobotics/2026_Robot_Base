package igknighters.subsystems.LimeLightVision;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightHelpers.PoseEstimate;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightHelpers.RawFiducial;
import org.junit.jupiter.api.Test;

class LimeLightVisionTest {
    static PoseEstimate est(double x, double y, double deg, int tags, double dist, double t) {
        return new PoseEstimate(
                new Pose2d(x, y, Rotation2d.fromDegrees(deg)),
                t,
                0,
                tags,
                0,
                dist,
                0,
                new RawFiducial[0],
                false);
    }

    final CameraConfig cfg = new CameraConfig("cam");

    @Test
    void rejects() {
        assertEquals("no tags", LimeLightVision.rejectReason(cfg, est(3, 3, 0, 0, 2, 1), 0));
        assertEquals(
                "spinning too fast", LimeLightVision.rejectReason(cfg, est(3, 3, 0, 1, 2, 1), 900));
        assertEquals("tags too far", LimeLightVision.rejectReason(cfg, est(3, 3, 0, 1, 9, 1), 0));
        assertEquals("off field", LimeLightVision.rejectReason(cfg, est(-3, 3, 0, 1, 2, 1), 0));
        assertNull(LimeLightVision.rejectReason(cfg, est(3, 3, 0, 1, 2, 1), 0));
    }

    @Test
    void stdDevsAndHeadingSource() {
        // single tag MT1: keep MT2 (gyro) heading, do not correct heading
        var m = LimeLightVision.measure(cfg, est(3, 4, 10, 1, 2, 1), est(9, 9, 50, 1, 2, 1));
        assertEquals(new Pose2d(3, 4, Rotation2d.fromDegrees(10)), m.pose());
        assertEquals(0.02 * 4 / 1, m.stdDevs().get(0, 0), 1e-9);
        assertEquals(LimeLightVision.NO_TRUST, m.stdDevs().get(2, 0));
        assertEquals(1, m.timestamp());

        // multi tag MT1 from the same frame: MT2 translation + MT1 heading
        m = LimeLightVision.measure(cfg, est(3, 4, 10, 2, 2, 1), est(9, 9, 50, 2, 2, 1.001));
        assertEquals(new Pose2d(3, 4, Rotation2d.fromDegrees(50)), m.pose());
        assertEquals(0.06 * 4 / 2, m.stdDevs().get(2, 0), 1e-9);

        // multi tag MT1 from a different frame: ignored
        m = LimeLightVision.measure(cfg, est(3, 4, 10, 2, 2, 1), est(9, 9, 50, 2, 2, 1.5));
        assertEquals(10, m.pose().getRotation().getDegrees(), 1e-9);
    }
}
