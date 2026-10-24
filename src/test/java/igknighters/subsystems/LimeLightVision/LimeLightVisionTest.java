package igknighters.subsystems.LimeLightVision;

import static igknighters.subsystems.LimeLightVision.LimeLightVision.NO_TRUST;
import static igknighters.subsystems.LimeLightVision.LimeLightVision.measure;
import static igknighters.subsystems.LimeLightVision.LimeLightVision.rejectReason;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightHelpers.PoseEstimate;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightHelpers.RawFiducial;
import org.junit.jupiter.api.Test;

class LimeLightVisionTest {
    static PoseEstimate est(double x, double y, double deg, int tags, double dist, double amb) {
        var fids = new RawFiducial[tags];
        for (int i = 0; i < tags; i++) fids[i] = new RawFiducial(19 + i, 0, 0, 0, dist, dist, amb);
        return new PoseEstimate(
                new Pose2d(x, y, Rotation2d.fromDegrees(deg)), 1, 0, tags, 0, dist, 0, fids, false);
    }

    static PoseEstimate at(PoseEstimate e, double t) {
        e.timestampSeconds = t;
        return e;
    }

    final CameraConfig cfg = new CameraConfig("cam");
    final PoseEstimate noMt1 = est(0, 0, 0, 0, 0, 0);

    @Test
    void rejects() {
        var one = est(3, 3, 0, 1, 2, 0);
        assertEquals("no tags", rejectReason(cfg, est(3, 3, 0, 0, 2, 0), noMt1, 0, 0));
        assertEquals("no MT1 in same frame", rejectReason(cfg, one, noMt1, 0, 0));
        assertEquals("spinning too fast", rejectReason(cfg, one, one, 900, 0));
        var far = est(3, 3, 0, 1, 9, 0);
        assertEquals("tags too far", rejectReason(cfg, far, far, 0, 0));
        var off = est(-3, 3, 0, 1, 2, 0);
        assertEquals("off field", rejectReason(cfg, off, off, 0, 0));
        assertNull(rejectReason(cfg, one, est(3.1, 3, 0, 1, 2, 0.5), 0, 0));
    }

    @Test
    void stdDevsAndHeadingSource() {
        // single tag MT1: keep MT2 (gyro) heading, do not correct heading
        var m = measure(cfg, est(3, 4, 10, 1, 2, 0), est(3, 4, 50, 1, 2, 0));
        assertEquals(new Pose2d(3, 4, Rotation2d.fromDegrees(10)), m.pose());
        assertEquals(0.02 * 4 / 1, m.stdDevs().get(0, 0), 1e-9);
        assertEquals(NO_TRUST, m.stdDevs().get(2, 0));

        // unambiguous multi tag MT1 from the same frame: MT2 translation + MT1 heading
        m = measure(cfg, est(3, 4, 10, 2, 2, 0.05), est(3.2, 4, 30, 2, 2, 0.05));
        assertEquals(new Pose2d(3, 4, Rotation2d.fromDegrees(30)), m.pose());
        assertEquals(0.06 * 4 / 2, m.stdDevs().get(2, 0), 1e-9);

        // multi tag MT1 from a different frame: ignored
        m = measure(cfg, est(3, 4, 10, 2, 2, 0.05), at(est(3, 4, 50, 2, 2, 0.05), 1.5));
        assertEquals(10, m.pose().getRotation().getDegrees(), 1e-9);
    }

    /** Seen on the robot: coplanar tags made MT1 flip between two headings frame to frame. */
    @Test
    void ambiguousMt1NeverTouchesHeading() {
        var m = measure(cfg, est(5.3, 4.3, 101, 2, 1, 0.6), est(5.43, 3.29, -98, 2, 1, 0.6));
        assertEquals(NO_TRUST, m.stdDevs().get(2, 0));
    }

    /** Seen on the robot: MT2 fails and reports field center (8.271, 4.035) with real tags. */
    @Test
    void mt2GarbageWithAmbiguousMt1IsRejected() {
        var mt2 = est(8.271, 4.035, 0, 2, 1, 0.6);
        var mt1 = est(5.44, 3.29, -97, 2, 1, 0.6);
        assertEquals("MT1/MT2 disagree", rejectReason(cfg, mt2, mt1, 0, 0));
    }

    /** Wrong heading (e.g. robot booted facing backwards): fix heading only, never jump x/y. */
    @Test
    void wrongHeadingWithTrustedMt1FixesHeadingOnly() {
        var mt2 = est(6.8, 4.1, 180, 2, 1, 0.05);
        var mt1 = est(5.3, 4.3, 0, 2, 1, 0.05);
        // MT1 has not held this heading long enough yet (could be a flip)
        assertEquals("MT1/MT2 disagree", rejectReason(cfg, mt2, mt1, 0, 0.5));
        assertNull(rejectReason(cfg, mt2, mt1, 0, 1.0));
        var m = measure(cfg, mt2, mt1);
        assertEquals(0, m.pose().getRotation().getDegrees(), 1e-9);
        assertEquals(NO_TRUST, m.stdDevs().get(0, 0));
        assertEquals(NO_TRUST, m.stdDevs().get(1, 0));
        assertTrue(m.stdDevs().get(2, 0) < 1);
    }

    /** A flip that happens to agree on x/y must not swing heading, only nudges are allowed. */
    @Test
    void agreeingFrameCannotSwingHeading() {
        var m = measure(cfg, est(5.3, 4.3, 100, 2, 1, 0.05), est(5.4, 4.3, -98, 2, 1, 0.05));
        assertEquals(100, m.pose().getRotation().getDegrees(), 1e-9);
        assertEquals(NO_TRUST, m.stdDevs().get(2, 0));
    }

    /** Gyro 180 off while the robot spins: the gap stays put, so a reset can happen mid-match. */
    @Test
    void headingGapIsStableWhileSpinningButNotWhenFlipping() {
        for (double robotDeg = 0; robotDeg < 360; robotDeg += 45) {
            var mt2 = est(5, 4, robotDeg, 2, 1, 0.05);
            var mt1 = est(5, 4, robotDeg + 180, 2, 1, 0.05);
            assertEquals(180, Math.abs(LimeLightVision.headingGap(mt2, mt1).getDegrees()), 1e-6);
        }
        var flipA = LimeLightVision.headingGap(est(5, 4, 0, 2, 1, 0), est(5, 4, 101, 2, 1, 0));
        var flipB = LimeLightVision.headingGap(est(5, 4, 0, 2, 1, 0), est(5, 4, -98, 2, 1, 0));
        assertTrue(Math.abs(flipA.minus(flipB).getDegrees()) > 5);
    }
}
