package igknighters.subsystems.LimeLightVision.ObjectDetection;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;

/**
 * Turns one label's noisy per-frame detections into steady objects.
 *
 * <p>Each object is a track with a position and a variance (a scalar Kalman filter). A detection
 * joins the track it is statistically closest to, within {@link #GATE_SIGMAS}; a far detection (big
 * variance) barely moves a track, a close one (small variance) snaps it. Detections that match
 * nothing start a new track. A track is reported once it has {@link #CONFIRM_HITS} hits and dropped
 * when nothing has matched it for {@link #MAX_AGE_SECONDS}.
 *
 * <p>ponytail: greedy nearest-match association with a round gate. Objects closer together than the
 * gate (piles of balls far away) merge into one track until the robot is close enough to tell them
 * apart. Upgrade path: an elongated (range vs bearing) gate, or Hungarian matching.
 */
class ObjectTracker {
    /** Match a detection to a track within this many combined standard deviations. */
    static final double GATE_SIGMAS = 3;

    /** Detections before a track is reported, so a one-frame false positive never shows up. */
    static final int CONFIRM_HITS = 3;

    /** Unmatched this long and the object is gone (left view, picked up, or never real). */
    static final double MAX_AGE_SECONDS = 0.5;

    /**
     * How fast an object may move (m^2/s). The knob between steady and responsive: at 0.01 a still
     * ball 4 m away from the back camera wanders under 7 cm per frame, but a ball rolling that far
     * away lags ~0.5 s behind (close balls ~0.1 s, their detections are trusted more).
     */
    static final double PROCESS_NOISE = 0.01;

    /** One detection and how far off it may be (1 std dev, meters). */
    record Measurement(Translation2d position, double stdDev) {}

    private static class Track {
        Translation2d position;
        double variance;
        double lastSeen;
        double lastPredict;
        int hits = 1;

        Track(Measurement m, double now) {
            position = m.position();
            variance = m.stdDev() * m.stdDev();
            lastSeen = now;
            lastPredict = now;
        }
    }

    private final List<Track> tracks = new ArrayList<>();

    /** One camera frame's detections of this label. */
    void update(List<Measurement> detections, double now) {
        for (Track t : tracks) {
            t.variance += PROCESS_NOISE * (now - t.lastPredict);
            t.lastPredict = now;
        }
        // every in-gate pair, best (fewest sigmas apart) first, each track and detection used once
        List<double[]> pairs = new ArrayList<>(); // {sigmas apart, track index, detection index}
        for (int i = 0; i < tracks.size(); i++) {
            Track t = tracks.get(i);
            for (int j = 0; j < detections.size(); j++) {
                Measurement m = detections.get(j);
                double sigma = Math.sqrt(t.variance + m.stdDev() * m.stdDev());
                double sigmas = t.position.getDistance(m.position()) / sigma;
                if (sigmas <= GATE_SIGMAS) {
                    pairs.add(new double[] {sigmas, i, j});
                }
            }
        }
        pairs.sort((a, b) -> Double.compare(a[0], b[0]));
        boolean[] trackUsed = new boolean[tracks.size()];
        boolean[] detectionUsed = new boolean[detections.size()];
        for (double[] p : pairs) {
            int i = (int) p[1];
            int j = (int) p[2];
            if (trackUsed[i] || detectionUsed[j]) {
                continue;
            }
            trackUsed[i] = true;
            detectionUsed[j] = true;
            Track t = tracks.get(i);
            Measurement m = detections.get(j);
            double r = m.stdDev() * m.stdDev();
            double k = t.variance / (t.variance + r);
            t.position = t.position.interpolate(m.position(), k);
            t.variance *= 1 - k;
            t.lastSeen = now;
            t.hits++;
        }
        for (int j = 0; j < detections.size(); j++) {
            if (!detectionUsed[j]) {
                tracks.add(new Track(detections.get(j), now));
            }
        }
    }

    /** Drop stale tracks and add the confirmed ones' positions to {@code out}. */
    void confirmed(double now, List<Translation2d> out) {
        tracks.removeIf(t -> now - t.lastSeen > MAX_AGE_SECONDS);
        for (Track t : tracks) {
            if (t.hits >= CONFIRM_HITS) {
                out.add(t.position);
            }
        }
    }
}
