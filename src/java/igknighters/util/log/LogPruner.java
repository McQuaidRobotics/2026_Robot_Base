package igknighters.util.log;

import java.io.File;
import java.util.Arrays;
import java.util.Comparator;

/** Keeps the log folder from filling the SD card. Run once at boot, before logging starts. */
public final class LogPruner {
    private LogPruner() {}

    static final long MAX_AGE_MS = 30L * 24 * 60 * 60 * 1000; // a month
    static final long MIN_FREE_BYTES = 5L * 1024 * 1024 * 1024; // 5 GB

    public static void prune(String dir) {
        prune(new File(dir), System.currentTimeMillis(), MAX_AGE_MS, MIN_FREE_BYTES);
    }

    /** Deletes logs older than maxAgeMs, then the oldest remaining until minFreeBytes are free. */
    static void prune(File dir, long nowMs, long maxAgeMs, long minFreeBytes) {
        File[] logs = dir.listFiles((d, name) -> name.endsWith(".wpilog"));
        if (logs == null) {
            return;
        }
        Arrays.sort(logs, Comparator.comparingLong(File::lastModified)); // oldest first
        // The roboRIO clock only gets set once the DS connects; a clock behind the files just
        // makes them look new, so nothing is wrongly deleted for age.
        for (File log : logs) {
            boolean tooOld = nowMs - log.lastModified() > maxAgeMs;
            boolean tooFull = dir.getUsableSpace() < minFreeBytes;
            if (!tooOld && !tooFull) {
                break;
            }
            log.delete();
        }
    }
}
