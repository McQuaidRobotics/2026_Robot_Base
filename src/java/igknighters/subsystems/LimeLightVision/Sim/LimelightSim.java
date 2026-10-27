package igknighters.subsystems.LimeLightVision.Sim;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import igknighters.subsystems.LimeLightVision.Helpers.LimelightHelpers;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Random;

// Credit: copied and trimmed from the YALL library's LimelightSim (which credits PhotonVision's
// VisionSystemSim for the design).
/**
 * Simulates one Limelight on the AprilTag pipeline by publishing the same NetworkTables keys a real
 * one does (botpose*, rawfiducials, tx/ty/ta/tv/tid, t2d, hb, stddevs, imu, ...), so robot code
 * runs unchanged in sim. Fed the TRUE robot pose every loop.
 *
 * <p>Poses are ground truth plus Gaussian noise, not a real solvePnP, so the real failure modes
 * (MT1 flips, MT2 reporting field center, high ambiguity) only appear via fault injection. The
 * "json" results entry is not published, so getLatestResults() is empty in sim.
 */
public class LimelightSim {

    /** NetworkTable of the Limelight being simulated. */
    private final NetworkTable table;

    /** Deterministic noise source. */
    private Random noise;

    /** Camera properties/noise/latency configuration. */
    private LimelightSimSettings settings;

    /** Transform from the robot's origin to the camera's lens. */
    private Transform3d robotToCamera = new Transform3d();

    /** AprilTag field layout used to source simulated targets. */
    private AprilTagFieldLayout fieldLayout;

    /** Recent true poses, so a frame reports where the robot was when it was captured. */
    private final TimeInterpolatableBuffer<Pose2d> truthHistory =
            TimeInterpolatableBuffer.createBuffer(1.0);

    /** Heartbeat counter, incremented once per published frame. */
    private long heartbeat = 0;

    private final NetworkTableEntry tvEntry;
    private final NetworkTableEntry txEntry;
    private final NetworkTableEntry tyEntry;
    private final NetworkTableEntry txncEntry;
    private final NetworkTableEntry tyncEntry;
    private final NetworkTableEntry taEntry;
    private final NetworkTableEntry tidEntry;
    private final NetworkTableEntry tlEntry;
    private final NetworkTableEntry clEntry;
    private final NetworkTableEntry tdistEntry;
    private final NetworkTableEntry hbEntry;
    private final NetworkTableEntry getpipeEntry;
    private final NetworkTableEntry getpipetypeEntry;
    private final NetworkTableEntry pipelineIndexEntry;
    private final DoubleArrayEntry t2dEntry;
    private final DoubleArrayEntry rawFiducialsEntry;
    private final DoubleArrayEntry targetPoseRobotSpaceEntry;
    private final DoubleArrayEntry targetPoseCameraSpaceEntry;
    private final DoubleArrayEntry cameraPoseTargetSpaceEntry;
    private final DoubleArrayEntry botPoseTargetSpaceEntry;
    private final DoubleArrayEntry cameraPoseRobotSpaceEntry;
    private final DoubleArrayEntry stddevsEntry;
    private final DoubleArrayEntry imuEntry;
    private final DoubleArrayEntry botposeEntry;
    private final DoubleArrayEntry botposeRedEntry;
    private final DoubleArrayEntry botposeBlueEntry;
    private final DoubleArrayEntry botposeOrbEntry;
    private final DoubleArrayEntry botposeOrbBlueEntry;
    private final DoubleArrayEntry botposeOrbRedEntry;
    private final DoubleArrayEntry robotOrientationSetEntry;

    /**
     * @param limelight Limelight name (NetworkTables table) to publish as.
     * @param robotToCamera Camera mount, see {@code CameraConfig.simMount}.
     */
    public LimelightSim(String limelight, Transform3d robotToCamera) {
        // Seed per camera: one shared seed made every camera publish identical noise, which the
        // estimator then counted as independent confirmations.
        this(limelight, new LimelightSimSettings().withRandomSeed(limelight.hashCode()));
        this.robotToCamera = robotToCamera;
    }

    public LimelightSim(String limelight, LimelightSimSettings settings) {
        this.table = NetworkTableInstance.getDefault().getTable(limelight);
        this.settings = settings;
        this.noise = new Random(settings.randomSeed);
        this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        tvEntry = table.getEntry("tv");
        txEntry = table.getEntry("tx");
        tyEntry = table.getEntry("ty");
        txncEntry = table.getEntry("txnc");
        tyncEntry = table.getEntry("tync");
        taEntry = table.getEntry("ta");
        tidEntry = table.getEntry("tid");
        tlEntry = table.getEntry("tl");
        clEntry = table.getEntry("cl");
        tdistEntry = table.getEntry("tdist");
        hbEntry = table.getEntry("hb");
        getpipeEntry = table.getEntry("getpipe");
        getpipetypeEntry = table.getEntry("getpipetype");
        pipelineIndexEntry = table.getEntry("pipeline");
        t2dEntry = table.getDoubleArrayTopic("t2d").getEntry(new double[0]);
        rawFiducialsEntry = table.getDoubleArrayTopic("rawfiducials").getEntry(new double[0]);
        targetPoseRobotSpaceEntry =
                table.getDoubleArrayTopic("targetpose_robotspace").getEntry(new double[0]);
        targetPoseCameraSpaceEntry =
                table.getDoubleArrayTopic("targetpose_cameraspace").getEntry(new double[0]);
        cameraPoseTargetSpaceEntry =
                table.getDoubleArrayTopic("camerapose_targetspace").getEntry(new double[0]);
        botPoseTargetSpaceEntry =
                table.getDoubleArrayTopic("botpose_targetspace").getEntry(new double[0]);
        cameraPoseRobotSpaceEntry =
                table.getDoubleArrayTopic("camerapose_robotspace").getEntry(new double[0]);
        stddevsEntry = table.getDoubleArrayTopic("stddevs").getEntry(new double[0]);
        imuEntry = table.getDoubleArrayTopic("imu").getEntry(new double[10]);
        botposeEntry = table.getDoubleArrayTopic("botpose").getEntry(new double[0]);
        botposeRedEntry = table.getDoubleArrayTopic("botpose_wpired").getEntry(new double[0]);
        botposeBlueEntry = table.getDoubleArrayTopic("botpose_wpiblue").getEntry(new double[0]);
        botposeOrbEntry = table.getDoubleArrayTopic("botpose_orb").getEntry(new double[0]);
        botposeOrbBlueEntry =
                table.getDoubleArrayTopic("botpose_orb_wpiblue").getEntry(new double[0]);
        botposeOrbRedEntry =
                table.getDoubleArrayTopic("botpose_orb_wpired").getEntry(new double[0]);
        robotOrientationSetEntry =
                table.getDoubleArrayTopic("robot_orientation_set").getEntry(new double[0]);
    }

    /**
     * Publish one simulated frame. Call every loop from simulationPeriodic with the TRUE robot pose
     * (never the estimator's pose, that feeds vision noise back into itself).
     */
    public void update(Pose2d robotPoseMeters) {
        if (!RobotBase.isSimulation()) {
            return;
        }
        double now = Timer.getFPGATimestamp();
        truthHistory.addSample(now, robotPoseMeters);
        double tlMs =
                Math.max(
                        0,
                        settings.avgPipelineLatencyMs
                                + noise.nextGaussian() * settings.latencyStdDevMs);
        // Limelight timestamps a frame as (publish time - latency), so publish the pose from then
        double latencySeconds = (tlMs + settings.avgCaptureLatencyMs) / 1000.0;
        Pose2d atCapture = truthHistory.getSample(now - latencySeconds).orElse(robotPoseMeters);
        publish(new Pose3d(atCapture), tlMs);
    }

    /** Project visible tags from {@code robotPose} and publish one frame. */
    private void publish(Pose3d robotPose, double tlMs) {

        Pose3d cameraPose = robotPose.plus(robotToCamera);

        List<TagObservation> visible = new ArrayList<>();

        for (AprilTag tag : fieldLayout.getTags()) {
            project(cameraPose, robotPose, tag).ifPresent(visible::add);
        }

        visible.sort((a, b) -> Double.compare(b.ta, a.ta));

        int tagCount = visible.size();
        TagObservation primary = tagCount > 0 ? visible.get(0) : null;

        double avgTagDist = 0;
        double avgTagArea = 0;
        double tagSpan = 0;

        for (TagObservation obs : visible) {
            avgTagDist += obs.distToCamera;
            avgTagArea += obs.ta;
        }

        if (tagCount > 0) {
            avgTagDist /= tagCount;
            avgTagArea /= tagCount;
        }

        for (int i = 0; i < visible.size(); i++) {
            for (int j = i + 1; j < visible.size(); j++) {
                tagSpan =
                        Math.max(
                                tagSpan,
                                visible.get(i)
                                        .pose
                                        .getTranslation()
                                        .getDistance(visible.get(j).pose.getTranslation()));
            }
        }

        double clMs = settings.avgCaptureLatencyMs;
        double totalLatencyMs = tlMs + clMs;

        double translationStdDev =
                tagCount > 0
                        ? settings.translationNoiseStdDevMeters
                                * (1 + avgTagDist)
                                / Math.sqrt(tagCount)
                        : 0;

        double rotationStdDev =
                tagCount > 0 ? settings.rotationNoiseStdDevDegrees / Math.sqrt(tagCount) : 0;

        Pose3d mt1Pose =
                tagCount > 0 ? addNoise(robotPose, translationStdDev, rotationStdDev) : robotPose;

        Pose3d mt2Pose = tagCount > 0 ? addNoise(robotPose, translationStdDev / 2.0, 0) : robotPose;

        double[] suppliedOrientation = robotOrientationSetEntry.get();

        if (suppliedOrientation.length >= 1) {
            Rotation3d groundTruthRotation = mt2Pose.getRotation();

            mt2Pose =
                    new Pose3d(
                            mt2Pose.getTranslation(),
                            new Rotation3d(
                                    groundTruthRotation.getX(),
                                    groundTruthRotation.getY(),
                                    Math.toRadians(suppliedOrientation[0])));
        }

        Pose3d mt1PoseRed = flipToRed(mt1Pose);
        Pose3d mt2PoseRed = flipToRed(mt2Pose);

        publishScalarEntries(primary, tagCount, tlMs, clMs);

        publishArrayEntries(
                robotPose,
                cameraPose,
                visible,
                primary,
                tagCount,
                tagSpan,
                avgTagDist,
                avgTagArea,
                totalLatencyMs,
                translationStdDev,
                rotationStdDev,
                mt1Pose,
                mt1PoseRed,
                mt2Pose,
                mt2PoseRed);

        heartbeat++;
    }

    /** Publish the scalar ("primary target") NT entries. */
    private void publishScalarEntries(
            TagObservation primary, int tagCount, double tlMs, double clMs) {
        tvEntry.setDouble(tagCount > 0 ? 1 : 0);
        txEntry.setDouble(primary != null ? primary.tx : 0);
        tyEntry.setDouble(primary != null ? primary.ty : 0);
        txncEntry.setDouble(primary != null ? primary.tx : 0);
        tyncEntry.setDouble(primary != null ? primary.ty : 0);
        taEntry.setDouble(primary != null ? primary.ta : 0);
        tidEntry.setDouble(primary != null ? primary.id : -1);
        tlEntry.setDouble(tlMs);
        clEntry.setDouble(clMs);
        tdistEntry.setDouble(primary != null ? primary.distToCamera : 0);
        hbEntry.setDouble(heartbeat);

        double pipelineIndex = pipelineIndexEntry.getDouble(0);

        getpipeEntry.setDouble(pipelineIndex);
        getpipetypeEntry.setString(settings.pipelineType);
    }

    /**
     * Publish the array-valued NT entries (rawfiducials, t2d, botpose family, target/camera
     * relative poses, stddevs, imu).
     */
    private void publishArrayEntries(
            Pose3d robotPose,
            Pose3d cameraPose,
            List<TagObservation> visible,
            TagObservation primary,
            int tagCount,
            double tagSpan,
            double avgTagDist,
            double avgTagArea,
            double totalLatencyMs,
            double translationStdDev,
            double rotationStdDev,
            Pose3d mt1Pose,
            Pose3d mt1PoseRed,
            Pose3d mt2Pose,
            Pose3d mt2PoseRed) {
        double[] rawFiducials = new double[7 * tagCount];

        for (int i = 0; i < visible.size(); i++) {
            TagObservation obs = visible.get(i);

            int base = i * 7;

            rawFiducials[base] = obs.id;
            rawFiducials[base + 1] = obs.tx;
            rawFiducials[base + 2] = obs.ty;
            rawFiducials[base + 3] = obs.ta;
            rawFiducials[base + 4] = obs.distToCamera;
            rawFiducials[base + 5] = obs.distToRobot;
            rawFiducials[base + 6] = obs.ambiguity;
        }

        rawFiducialsEntry.set(rawFiducials);

        botposeEntry.set(
                botPoseArray(
                        mt1Pose,
                        totalLatencyMs,
                        tagCount,
                        tagSpan,
                        avgTagDist,
                        avgTagArea,
                        rawFiducials));

        botposeBlueEntry.set(
                botPoseArray(
                        mt1Pose,
                        totalLatencyMs,
                        tagCount,
                        tagSpan,
                        avgTagDist,
                        avgTagArea,
                        rawFiducials));

        botposeRedEntry.set(
                botPoseArray(
                        mt1PoseRed,
                        totalLatencyMs,
                        tagCount,
                        tagSpan,
                        avgTagDist,
                        avgTagArea,
                        rawFiducials));

        botposeOrbEntry.set(
                botPoseArray(
                        mt2Pose,
                        totalLatencyMs,
                        tagCount,
                        tagSpan,
                        avgTagDist,
                        avgTagArea,
                        rawFiducials));

        botposeOrbBlueEntry.set(
                botPoseArray(
                        mt2Pose,
                        totalLatencyMs,
                        tagCount,
                        tagSpan,
                        avgTagDist,
                        avgTagArea,
                        rawFiducials));

        botposeOrbRedEntry.set(
                botPoseArray(
                        mt2PoseRed,
                        totalLatencyMs,
                        tagCount,
                        tagSpan,
                        avgTagDist,
                        avgTagArea,
                        rawFiducials));

        double[] t2d =
                new double[] {
                    tagCount > 0 ? 1 : 0,
                    tagCount,
                    totalLatencyMs,
                    0,
                    primary != null ? primary.tx : 0,
                    primary != null ? primary.ty : 0,
                    primary != null ? primary.tx : 0,
                    primary != null ? primary.ty : 0,
                    primary != null ? primary.ta : 0,
                    primary != null ? primary.id : -1,
                    -1,
                    -1,
                    primary != null ? primary.apparentWidthPixels : 0,
                    primary != null ? primary.apparentWidthPixels : 0,
                    primary != null ? primary.apparentWidthPixels : 0,
                    primary != null ? primary.apparentWidthPixels : 0,
                    0
                };

        t2dEntry.set(t2d);

        if (primary != null) {
            Pose3d tagPose = primary.pose;

            targetPoseRobotSpaceEntry.set(
                    LimelightHelpers.pose3dToArray(tagPose.relativeTo(robotPose)));

            targetPoseCameraSpaceEntry.set(
                    LimelightHelpers.pose3dToArray(tagPose.relativeTo(cameraPose)));

            cameraPoseTargetSpaceEntry.set(
                    LimelightHelpers.pose3dToArray(cameraPose.relativeTo(tagPose)));
            botPoseTargetSpaceEntry.set(
                    LimelightHelpers.pose3dToArray(robotPose.relativeTo(tagPose)));
        } else {
            targetPoseRobotSpaceEntry.set(new double[0]);
            targetPoseCameraSpaceEntry.set(new double[0]);
            cameraPoseTargetSpaceEntry.set(new double[0]);
            botPoseTargetSpaceEntry.set(new double[0]);
        }

        cameraPoseRobotSpaceEntry.set(
                LimelightHelpers.pose3dToArray(
                        new Pose3d(robotToCamera.getTranslation(), robotToCamera.getRotation())));

        stddevsEntry.set(
                new double[] {
                    translationStdDev,
                    translationStdDev,
                    translationStdDev,
                    rotationStdDev,
                    rotationStdDev,
                    rotationStdDev,
                    translationStdDev / 2.0,
                    translationStdDev / 2.0,
                    translationStdDev / 2.0,
                    0,
                    0,
                    0
                });

        double robotYawDeg = Math.toDegrees(robotPose.getRotation().getZ());
        double robotPitchDeg = Math.toDegrees(robotPose.getRotation().getY());
        double robotRollDeg = Math.toDegrees(robotPose.getRotation().getX());

        imuEntry.set(
                new double[] {
                    robotYawDeg, robotRollDeg, robotPitchDeg, robotYawDeg, 0, 0, 0, 0, 0, 0
                });
    }

    /**
     * Build the flat "botpose*" NT array: [x,y,z,roll,pitch,yaw, latency, tagCount, tagSpan,
     * avgDist, avgArea, (id,txnc,tync,ta,distToCamera,distToRobot,ambiguity)*tagCount].
     */
    private double[] botPoseArray(
            Pose3d pose,
            double latencyMs,
            int tagCount,
            double tagSpan,
            double avgTagDist,
            double avgTagArea,
            double[] rawFiducials) {
        if (tagCount == 0) {
            return new double[0];
        }

        double[] pose6 = LimelightHelpers.pose3dToArray(pose);
        double[] result = new double[11 + rawFiducials.length];

        System.arraycopy(pose6, 0, result, 0, 6);

        result[6] = latencyMs;
        result[7] = tagCount;
        result[8] = tagSpan;
        result[9] = avgTagDist;
        result[10] = avgTagArea;

        System.arraycopy(rawFiducials, 0, result, 11, rawFiducials.length);

        return result;
    }

    /**
     * Flip a blue-alliance-origin pose to the equivalent red-alliance-origin pose (180 degree
     * rotation about the field center).
     */
    private Pose3d flipToRed(Pose3d bluePose) {
        Rotation3d rotation = bluePose.getRotation();

        return new Pose3d(
                fieldLayout.getFieldLength() - bluePose.getX(),
                fieldLayout.getFieldWidth() - bluePose.getY(),
                bluePose.getZ(),
                new Rotation3d(rotation.getX(), rotation.getY(), rotation.getZ() + Math.PI));
    }

    /** Apply Gaussian noise to a pose's translation and yaw. */
    private Pose3d addNoise(Pose3d pose, double translationStdDev, double rotationStdDevDegrees) {
        if (translationStdDev <= 0 && rotationStdDevDegrees <= 0) {
            return pose;
        }

        double x = pose.getX() + noise.nextGaussian() * translationStdDev;

        double y = pose.getY() + noise.nextGaussian() * translationStdDev;

        double z = pose.getZ() + noise.nextGaussian() * translationStdDev * 0.5;

        Rotation3d rotation = pose.getRotation();

        double yaw = rotation.getZ() + Math.toRadians(noise.nextGaussian() * rotationStdDevDegrees);

        return new Pose3d(x, y, z, new Rotation3d(rotation.getX(), rotation.getY(), yaw));
    }

    /**
     * Project a field {@link AprilTag} into the simulated camera, returning an observation if it is
     * within the configured field of view, range and incidence angle.
     */
    private Optional<TagObservation> project(Pose3d cameraPose, Pose3d robotPose, AprilTag tag) {
        Pose3d tagPose = tag.pose;

        Transform3d camToTag = new Transform3d(cameraPose, tagPose);

        double x = camToTag.getX();
        double y = camToTag.getY();
        double z = camToTag.getZ();

        if (x <= 0.02) {
            return Optional.empty();
        }

        double distance = camToTag.getTranslation().getNorm();

        if (distance > settings.maxDetectionRangeMeters) {
            return Optional.empty();
        }

        double halfHFovDeg = settings.horizontalFOV.getDegrees() / 2.0;

        double halfVFovDeg = settings.verticalFOV.getDegrees() / 2.0;

        // tx positive = target right of crosshair
        // ty positive = target below crosshair
        // Limelight convention.
        double yawDeg = Math.toDegrees(Math.atan2(-y, x));

        double pitchDeg = Math.toDegrees(Math.atan2(-z, Math.hypot(x, y)));

        if (Math.abs(yawDeg) > halfHFovDeg || Math.abs(pitchDeg) > halfVFovDeg) {
            return Optional.empty();
        }

        Translation3d tagNormal = new Translation3d(1, 0, 0).rotateBy(tagPose.getRotation());

        Translation3d tagToCam = cameraPose.getTranslation().minus(tagPose.getTranslation());

        double dot =
                tagNormal.getX() * tagToCam.getX()
                        + tagNormal.getY() * tagToCam.getY()
                        + tagNormal.getZ() * tagToCam.getZ();

        double incidenceDeg =
                Math.toDegrees(Math.acos(MathUtil.clamp(dot / tagToCam.getNorm(), -1, 1)));

        if (incidenceDeg > settings.maxTagIncidenceAngleDegrees) {
            return Optional.empty();
        }

        double fx = (settings.resolutionWidth / 2.0) / Math.tan(Math.toRadians(halfHFovDeg));

        double apparentWidthPx =
                fx * settings.tagSizeMeters * Math.cos(Math.toRadians(incidenceDeg)) / x;

        double areaPx = apparentWidthPx * apparentWidthPx;

        double ta =
                MathUtil.clamp(
                        areaPx / (settings.resolutionWidth * settings.resolutionHeight) * 100.0,
                        0,
                        100);

        double noisyYaw = yawDeg + noise.nextGaussian() * settings.angleNoiseStdDevDegrees;

        double noisyPitch = pitchDeg + noise.nextGaussian() * settings.angleNoiseStdDevDegrees;

        double distToRobot = robotPose.getTranslation().getDistance(tagPose.getTranslation());

        double ambiguity =
                MathUtil.clamp(noise.nextDouble() * 0.05 + (incidenceDeg / 90.0) * 0.15, 0, 1);

        return Optional.of(
                new TagObservation(
                        tag.ID,
                        tagPose,
                        ta,
                        noisyYaw,
                        noisyPitch,
                        distance,
                        distToRobot,
                        ambiguity,
                        apparentWidthPx));
    }

    /** A single simulated tag detection. */
    private static class TagObservation {

        final int id;
        final Pose3d pose;
        final double ta;
        final double tx;
        final double ty;
        final double distToCamera;
        final double distToRobot;
        final double ambiguity;
        final double apparentWidthPixels;

        TagObservation(
                int id,
                Pose3d pose,
                double ta,
                double tx,
                double ty,
                double distToCamera,
                double distToRobot,
                double ambiguity,
                double apparentWidthPixels) {
            this.id = id;
            this.pose = pose;
            this.ta = ta;
            this.tx = tx;
            this.ty = ty;
            this.distToCamera = distToCamera;
            this.distToRobot = distToRobot;
            this.ambiguity = ambiguity;
            this.apparentWidthPixels = apparentWidthPixels;
        }
    }
}
