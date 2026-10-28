package igknighters.constants;

import igknighters.subsystems.LimeLightVision.CameraConfig;
import igknighters.subsystems.LimeLightVision.ObjectDetection.ObjectDetectionConfig;
import igknighters.subsystems.swerve.swerveconstants.CommonSwerveConsts;
import java.util.List;

/**
 * Per robot constants contract.
 *
 * <p>The robot base declares only season independent hardware: the drivetrain and the vision
 * cameras. A season repository extends this with the accessors its superstructure needs.
 */
public abstract class RobotConsts {

    public abstract SWERVE_CONSTS swerve();

    public abstract boolean disableAllLogs();

    public abstract kLimelightVisionConsts limelightVision();

    public interface SWERVE_CONSTS {
        CommonSwerveConsts getCommonSwerveConsts();
    }

    /**
     * Limelights on this robot. Names must match the hostname configured on each physical
     * Limelight.
     */
    public interface kLimelightVisionConsts {
        /** Localization cameras. */
        List<CameraConfig> cameras();

        /** Neural detector cameras. A camera is in one list or the other, never both. */
        List<ObjectDetectionConfig> objectDetectionCameras();

        boolean disableVisionLogs();
    }
}
