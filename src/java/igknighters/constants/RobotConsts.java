package igknighters.constants;

import edu.wpi.first.math.geometry.Pose3d;
import igknighters.subsystems.swerve.swerveconstants.CommonSwerveConsts;

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
     * Limelight hostnames by mounting position.
     *
     * <p>The returned strings must match the hostname configured on each physical Limelight.
     */
    public interface kLimelightVisionConsts {
        String primaryCam();

        Pose3d primaryCamPose();

        String frontCam();

        Pose3d frontCamPose();

        String backCam();

        Pose3d backCamPose();

        String leftCam();

        Pose3d leftCamPose();

        boolean disableVisionLogs();
    }
}
