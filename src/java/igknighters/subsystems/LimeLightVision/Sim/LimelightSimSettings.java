package igknighters.subsystems.LimeLightVision.Sim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * Configurable properties describing a simulated Limelight camera, used by {@link LimelightSim}.
 *
 * <p>The defaults approximate a Limelight 3 running the AprilTag pipeline at 1280x960 and require
 * no configuration to get a usable simulation running. Construct one with {@code new
 * LimelightSimSettings()} and tweak only the fields that matter for your camera with the {@code
 * withXXX} methods.
 */
public class LimelightSimSettings {

    /** Horizontal sensor resolution in pixels. */
    double resolutionWidth = 1280;

    /** Vertical sensor resolution in pixels. */
    double resolutionHeight = 960;

    /** Horizontal field of view of the camera. */
    Rotation2d horizontalFOV = Rotation2d.fromDegrees(63.3);

    /** Vertical field of view of the camera. */
    Rotation2d verticalFOV = Rotation2d.fromDegrees(49.7);

    /**
     * Physical size of the AprilTag (black square, edge-to-edge) in meters. Defaults to the 6.5in
     * tags used in modern FRC games.
     */
    double tagSizeMeters = Units.inchesToMeters(6.5);

    /** Maximum distance a tag may be detected from, in meters. */
    double maxDetectionRangeMeters = 6.0;

    /**
     * Maximum angle of incidence between the camera's line of sight and a tag's face before the tag
     * is considered too edge-on to reliably detect, in degrees.
     */
    double maxTagIncidenceAngleDegrees = 80;

    /** Average pipeline processing latency ("tl"), in milliseconds. */
    double avgPipelineLatencyMs = 40;

    /** Standard deviation of the pipeline processing latency, in milliseconds. */
    double latencyStdDevMs = 8;

    /** Average capture latency ("cl"), in milliseconds. */
    double avgCaptureLatencyMs = 10;

    /** Standard deviation applied to tx/ty readings, in degrees. */
    double angleNoiseStdDevDegrees = 0.15;

    /**
     * Baseline standard deviation applied to the translation component of botpose readings, in
     * meters, for a single tag seen at 1 meter. Scaled down as more tags are seen and up with
     * average tag distance.
     *
     * <p>0.004 is measured: limelight-back MT2 frame-to-frame noise in the Oct 2026 robot logs was
     * 0.2-0.6 cm at 0.5-2 m (YALL's 0.03 default was ~10x too noisy). Rotation noise already
     * matched the measured ~1 deg MT1 heading noise.
     */
    double translationNoiseStdDevMeters = 0.004;

    /**
     * Baseline standard deviation applied to the rotation component of botpose readings, in
     * degrees, for a single tag. Scaled down as more tags are seen.
     */
    double rotationNoiseStdDevDegrees = 1.5;

    // ---- Fault injection, modeled on what the Oct 2026 robot logs actually showed ----

    /**
     * Chance per frame that MT1 starts flipping to its mirror solution (heading about +-105 deg,
     * position about 1 m off, ambiguity still LOW, which is what makes it dangerous).
     */
    double mt1FlipStartChance = 0.02;

    /** Chance per frame a flip ends. 0.15 gives runs of ~7 frames; the robot saw up to 30. */
    double mt1FlipEndChance = 0.15;

    /** Chance per tag per frame of a high (0.3-0.95) ambiguity score. */
    double ambiguitySpikeChance = 0.05;

    /** Chance per frame MT1 is not updated with the frame (~2% on the robot). */
    double staleMt1Chance = 0.02;

    /**
     * MT2 is solved from the heading we send. Beyond this heading error (degrees) it fails and
     * reports field center, like the real camera; below it the position swings around the tag.
     */
    double mt2FailHeadingErrorDeg = 30;

    /** Chance per frame the camera drops out (heartbeat stops). ~Once per 2 min at 50 fps. */
    double disconnectChance = 1.0 / 6000;

    /** How long a dropout lasts, in seconds. */
    double disconnectSeconds = 1.5;

    /**
     * Set the fault injection rates. All chances are per frame (per tag for ambiguity).
     *
     * @return {@link LimelightSimSettings} for chaining.
     */
    public LimelightSimSettings withFaults(
            double mt1FlipStartChance,
            double mt1FlipEndChance,
            double ambiguitySpikeChance,
            double staleMt1Chance,
            double disconnectChance) {
        this.mt1FlipStartChance = mt1FlipStartChance;
        this.mt1FlipEndChance = mt1FlipEndChance;
        this.ambiguitySpikeChance = ambiguitySpikeChance;
        this.staleMt1Chance = staleMt1Chance;
        this.disconnectChance = disconnectChance;
        return this;
    }

    /** Seed used for the deterministic noise {@link java.util.Random} source. */
    long randomSeed = 4152026L;

    /** Pipeline type string reported by "getpipetype". */
    String pipelineType = "pipe_fiducial";

    /** Construct {@link LimelightSimSettings} with defaults approximating a Limelight 3. */
    public LimelightSimSettings() {}

    /**
     * Set the simulated sensor resolution.
     *
     * @param widthPixels Horizontal resolution in pixels.
     * @param heightPixels Vertical resolution in pixels.
     * @return {@link LimelightSimSettings} for chaining.
     */
    public LimelightSimSettings withResolution(double widthPixels, double heightPixels) {
        resolutionWidth = widthPixels;
        resolutionHeight = heightPixels;
        return this;
    }

    /**
     * Set the simulated camera's field of view.
     *
     * @param horizontalDegrees Horizontal field of view in degrees.
     * @param verticalDegrees Vertical field of view in degrees.
     * @return {@link LimelightSimSettings} for chaining.
     */
    public LimelightSimSettings withFOV(double horizontalDegrees, double verticalDegrees) {
        horizontalFOV = Rotation2d.fromDegrees(horizontalDegrees);
        verticalFOV = Rotation2d.fromDegrees(verticalDegrees);
        return this;
    }

    /**
     * Set the physical size of the AprilTags being detected.
     *
     * @param sizeMeters Edge-to-edge size of the tag's black square, in meters.
     * @return {@link LimelightSimSettings} for chaining.
     */
    public LimelightSimSettings withTagSize(double sizeMeters) {
        tagSizeMeters = sizeMeters;
        return this;
    }

    /**
     * Set the maximum distance a tag may be detected from.
     *
     * @param rangeMeters Maximum detection range, in meters.
     * @return {@link LimelightSimSettings} for chaining.
     */
    public LimelightSimSettings withMaxDetectionRange(double rangeMeters) {
        maxDetectionRangeMeters = rangeMeters;
        return this;
    }

    /**
     * Set the maximum angle of incidence a tag may be viewed at before it is considered too edge-on
     * to detect.
     *
     * @param degrees Maximum incidence angle, in degrees.
     * @return {@link LimelightSimSettings} for chaining.
     */
    public LimelightSimSettings withMaxTagIncidenceAngle(double degrees) {
        maxTagIncidenceAngleDegrees = degrees;
        return this;
    }

    /**
     * Set the simulated pipeline processing latency ("tl").
     *
     * @param avgMs Average latency, in milliseconds.
     * @param stdDevMs Standard deviation of the latency, in milliseconds.
     * @return {@link LimelightSimSettings} for chaining.
     */
    public LimelightSimSettings withPipelineLatency(double avgMs, double stdDevMs) {
        avgPipelineLatencyMs = avgMs;
        latencyStdDevMs = stdDevMs;
        return this;
    }

    /**
     * Set the simulated capture latency ("cl").
     *
     * @param avgMs Average capture latency, in milliseconds.
     * @return {@link LimelightSimSettings} for chaining.
     */
    public LimelightSimSettings withCaptureLatency(double avgMs) {
        avgCaptureLatencyMs = avgMs;
        return this;
    }

    /**
     * Set the noise applied to tx/ty/txnc/tync readings.
     *
     * @param stdDevDegrees Standard deviation of the angular noise, in degrees.
     * @return {@link LimelightSimSettings} for chaining.
     */
    public LimelightSimSettings withAngleNoise(double stdDevDegrees) {
        angleNoiseStdDevDegrees = stdDevDegrees;
        return this;
    }

    /**
     * Set the noise applied to botpose translation and rotation readings, for a single tag seen at
     * 1 meter.
     *
     * @param translationStdDevMeters Standard deviation of the translation noise, in meters.
     * @param rotationStdDevDegrees Standard deviation of the rotation noise, in degrees.
     * @return {@link LimelightSimSettings} for chaining.
     */
    public LimelightSimSettings withPoseNoise(
            double translationStdDevMeters, double rotationStdDevDegrees) {
        translationNoiseStdDevMeters = translationStdDevMeters;
        rotationNoiseStdDevDegrees = rotationStdDevDegrees;
        return this;
    }

    /**
     * Set the seed used for the deterministic noise source.
     *
     * @param seed Random seed.
     * @return {@link LimelightSimSettings} for chaining.
     */
    public LimelightSimSettings withRandomSeed(long seed) {
        randomSeed = seed;
        return this;
    }

    /**
     * Set the pipeline type string reported by "getpipetype".
     *
     * @param type Pipeline type, e.g. "pipe_fiducial".
     * @return {@link LimelightSimSettings} for chaining.
     */
    public LimelightSimSettings withPipelineType(String type) {
        pipelineType = type;
        return this;
    }

    /**
     * Create {@link LimelightSimSettings} with all noise, latency and random faults zeroed out,
     * useful for deterministic testing. MT2 still fails on a large heading error, since that is how
     * MegaTag2 works rather than a random fault.
     *
     * @return {@link LimelightSimSettings} with no noise, latency or random faults.
     */
    public static LimelightSimSettings perfect() {
        return new LimelightSimSettings()
                .withPipelineLatency(0, 0)
                .withCaptureLatency(0)
                .withAngleNoise(0)
                .withPoseNoise(0, 0)
                .withFaults(0, 1, 0, 0, 0);
    }
}
