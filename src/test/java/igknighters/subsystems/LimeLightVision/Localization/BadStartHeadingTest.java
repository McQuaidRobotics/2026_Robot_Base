package igknighters.subsystems.LimeLightVision.Localization;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import igknighters.subsystems.LimeLightVision.CameraConfig;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightHelpers.PoseEstimate;
import igknighters.subsystems.LimeLightVision.Localization.LocalizationSim.LimelightSim;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

/**
 * Closed loop: the camera sim (all faults on) and the real vision decisions, frame by frame like
 * periodic, feeding WPILib's pose estimator (CTRE's works the same, same default trust).
 */
class BadStartHeadingTest {
    @BeforeAll
    static void setup() {
        assertTrue(HAL.initialize(500, 0));
        SimHooks.pauseTiming();
    }

    @AfterAll
    static void teardown() {
        SimHooks.resumeTiming();
    }

    record Result(
            double fixedAt,
            double finalHeadingErr,
            double worstXy,
            double worstHeadingAfterFix,
            int resets) {}

    static int nextCam = 0;

    /**
     * Robot sits 2.5 m in front of tags 19/20 (seen by limelight-back), spinning in place at {@code
     * spinDps}. The estimator starts {@code startErrorDeg} off the true heading.
     */
    static Result run(double startErrorDeg, double spinDps, double seconds) {
        Pose2d tag19 =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)
                        .getTagPose(19)
                        .orElseThrow()
                        .toPose2d();
        Translation2d spot = new Translation2d(tag19.getX() + 2.5, tag19.getY() + 0.18);
        var cfg =
                new CameraConfig("closedloop" + nextCam++)
                        .simMount(-0.3129, -0.2237, 0.1994, 15, 180); // real limelight-back mount
        var ll = new Limelight(cfg.name);
        var sim = new LimelightSim(cfg.name, cfg.simMount); // default settings: all faults on
        var tracker = new LocalizationVision.HeadingGapTracker();
        TimeInterpolatableBuffer<Rotation2d> sentHeadings =
                TimeInterpolatableBuffer.createBuffer(1.5);

        var kinematics =
                new SwerveDriveKinematics(
                        new Translation2d(0.3, 0.3),
                        new Translation2d(0.3, -0.3),
                        new Translation2d(-0.3, 0.3),
                        new Translation2d(-0.3, -0.3));
        var wheels = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) wheels[i] = new SwerveModulePosition();
        Rotation2d trueHeading = Rotation2d.kZero;
        var est =
                new SwerveDrivePoseEstimator(
                        kinematics,
                        trueHeading, // the gyro measures real rotation; the offset is the error
                        wheels,
                        new Pose2d(spot, Rotation2d.fromDegrees(startErrorDeg)));

        double lastFrame = -1, fixedAt = -1, worstXy = 0, worstAfterFix = 0;
        double start = Timer.getFPGATimestamp();
        int resets = 0;
        for (int loop = 0; loop < 50 * seconds; loop++) {
            SimHooks.stepTiming(0.02);
            double now = Timer.getFPGATimestamp();
            trueHeading = trueHeading.plus(Rotation2d.fromDegrees(spinDps * 0.02));
            Pose2d truth = new Pose2d(spot, trueHeading);
            est.updateWithTime(now, trueHeading, wheels); // spinning in place: wheels go nowhere
            Pose2d pose = est.getEstimatedPosition();
            sentHeadings.addSample(now, pose.getRotation());

            // same order as the robot: send heading, camera publishes, read it back
            ll.setRobotOrientation(pose.getRotation().getDegrees(), spinDps);
            sim.update(truth);
            PoseEstimate mt2 = ll.getMegaTag2();
            LocalizationVision.alignMt2ToCaptureHeading(
                    mt2, sentHeadings.getSample(mt2.timestampSeconds).orElse(pose.getRotation()));
            if (mt2.timestampSeconds == lastFrame) continue;
            lastFrame = mt2.timestampSeconds;
            PoseEstimate mt1 = ll.getMegaTag1();
            var d = LocalizationVision.decide(cfg, mt2, mt1, spinDps, tracker);
            if (d.reason() == null) {
                var m = d.measurement();
                if (m.headingReset()) { // same as periodic: shift heading by the voted gap
                    resets++;
                    Pose2d p = est.getEstimatedPosition();
                    est.resetPose(
                            new Pose2d(p.getTranslation(), p.getRotation().plus(m.headingGap())));
                    tracker.clear();
                } else {
                    est.addVisionMeasurement(m.pose(), m.timestamp(), m.stdDevs());
                }
            }

            pose = est.getEstimatedPosition();
            worstXy = Math.max(worstXy, pose.getTranslation().getDistance(spot));
            double headingErr = Math.abs(pose.getRotation().minus(trueHeading).getDegrees());
            if (fixedAt < 0 && headingErr < 2) fixedAt = now - start;
            if (fixedAt >= 0) worstAfterFix = Math.max(worstAfterFix, headingErr);
        }
        double finalErr = est.getEstimatedPosition().getRotation().minus(trueHeading).getDegrees();
        var r = new Result(fixedAt, finalErr, worstXy, worstAfterFix, resets);
        System.out.printf(
                "CLOSEDLOOP start error %.0f deg, spin %.0f deg/s: heading fixed after %.2f s,"
                        + " final heading error %.2f deg, worst heading error once fixed %.2f deg,"
                        + " worst x/y error %.3f m, heading resets %d%n",
                startErrorDeg,
                spinDps,
                r.fixedAt(),
                r.finalHeadingErr(),
                r.worstHeadingAfterFix(),
                r.worstXy(),
                r.resets());
        return r;
    }

    /** Robot really faces 0 deg but booted believing otherwise. */
    @ParameterizedTest
    @ValueSource(doubles = {15, 45, 90, 180, -30})
    void wrongStartHeadingIsFixedWithoutMovingPosition(double startErrorDeg) {
        var r = run(startErrorDeg, 0, 20);
        assertTrue(r.fixedAt() >= 0 && r.fixedAt() < 5, "heading not fixed within 5 s");
        assertEquals(0, r.finalHeadingErr(), 2);
        assertTrue(r.worstXy() < 0.25, "position was dragged while heading was wrong");
    }

    /**
     * Correct heading, spinning in place. MT2 solves with a heading newer than its image, which
     * used to bend its position and fool the heading vote into "fixing" a good heading.
     */
    @ParameterizedTest
    @ValueSource(doubles = {90, 180, 360})
    void spinningInPlaceDoesNotWanderOrBreakHeading(double spinDps) {
        var r = run(0, spinDps, 20);
        assertEquals(0, r.resets(), "heading was 'fixed' while it was right");
        assertTrue(r.worstHeadingAfterFix() < 2, "heading wandered while spinning");
        assertTrue(r.worstXy() < 0.05, "position wandered while spinning: " + r.worstXy());
    }

    /** Booted wrong AND spinning: the vote still works mid-spin (gap is steady while turning). */
    @Test
    void wrongHeadingIsFixedWhileSpinning() {
        var r = run(45, 180, 20);
        assertTrue(r.fixedAt() >= 0 && r.fixedAt() < 5, "heading not fixed within 5 s");
        assertEquals(0, r.finalHeadingErr(), 2);
        assertTrue(r.worstXy() < 0.25, "position was dragged while heading was wrong");
    }
}
