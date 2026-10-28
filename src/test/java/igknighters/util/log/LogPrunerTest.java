package igknighters.util.log;

import static org.junit.jupiter.api.Assertions.*;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class LogPrunerTest {
    static final long DAY = 24L * 60 * 60 * 1000;

    static File log(Path dir, String name, long modifiedMs) throws Exception {
        File f = Files.createFile(dir.resolve(name)).toFile();
        f.setLastModified(modifiedMs);
        return f;
    }

    @Test
    void dropsMonthOldLogsOnly(@TempDir Path dir) throws Exception {
        long now = 100 * DAY;
        File old = log(dir, "old.wpilog", now - 40 * DAY);
        File recent = log(dir, "recent.wpilog", now - 2 * DAY);
        File other = log(dir, "notes.txt", now - 40 * DAY);
        LogPruner.prune(dir.toFile(), now, LogPruner.MAX_AGE_MS, 0);
        assertFalse(old.exists());
        assertTrue(recent.exists());
        assertTrue(other.exists(), "only .wpilog files are touched");
    }

    @Test
    void lowSpaceDropsOldestFirst(@TempDir Path dir) throws Exception {
        long now = 100 * DAY;
        File a = log(dir, "a.wpilog", now - 3 * DAY);
        File b = log(dir, "b.wpilog", now - 2 * DAY);
        // free space can never reach Long.MAX_VALUE, so it deletes everything, oldest first
        LogPruner.prune(dir.toFile(), now, LogPruner.MAX_AGE_MS, Long.MAX_VALUE);
        assertFalse(a.exists());
        assertFalse(b.exists());
    }
}
