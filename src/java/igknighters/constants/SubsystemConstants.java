package igknighters.constants;

/**
 * Static subsystem configuration for the robot base.
 *
 * <p>Only season independent hardware lives here. Motor IDs, gearing and gains for game specific
 * superstructure mechanisms belong in a season repository.
 */
public class SubsystemConstants {

    public static final boolean disableAllLogs = false;

    /**
     * Limelight hostnames by mounting position.
     *
     * <p>These must match the hostname configured on each physical Limelight.
     */
    public static class kLimelightVision {
        public static final String primaryCam = "limelight";
        public static final String frontCam = "limelight-intake";
        public static final String backCam = "limelight-back";
        public static final String rightCam = "limelight-left";
        public static boolean disableVisionLogs = true;

        static {
            if (disableAllLogs) {
                disableVisionLogs = true;
            }
        }
    }
}
