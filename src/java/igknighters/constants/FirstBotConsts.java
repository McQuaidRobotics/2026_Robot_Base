package igknighters.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import igknighters.subsystems.swerve.swerveconstants.CommonSwerveConsts;
import igknighters.subsystems.swerve.swerveconstants.SwerveConsts;

public class FirstBotConsts extends RobotConsts {

    public static final boolean disableAllLogs = false;

    @Override
    public SWERVE_CONSTS swerve() {
        return new SwerveConstsProvider();
    }

    public static class SwerveConstsProvider implements SWERVE_CONSTS {
        static SwerveConsts swerveConsts = new SwerveConsts();

        static CommonSwerveConsts commonSwerveConsts = swerveConsts.getSwerveConsts();

        @Override
        public CommonSwerveConsts getCommonSwerveConsts() {
            return commonSwerveConsts;
        }
    }

    @Override
    public boolean disableAllLogs() {
        return disableAllLogs;
    }

    @Override
    public kLimelightVisionConsts limelightVision() {
        return new FirstBotLimelightVisionConsts();
    }

    public static class FirstBotLimelightVisionConsts implements kLimelightVisionConsts {
        @Override
        public String primaryCam() {
            return "limelight";
        }

        @Override
        public Pose3d primaryCamPose() {
            return new Pose3d(
                    0.2991612,
                    -0.2966466,
                    0.321437,
                    new Rotation3d(0.0, Math.toRadians(15.0), Math.toRadians(225.0)));
        }

        @Override
        public String frontCam() {
            return "limelight-intake";
        }

        @Override
        public Pose3d frontCamPose() {
            return new Pose3d(
                    0.3078734,
                    0.0370078,
                    0.2056638,
                    new Rotation3d(0.0, Math.toRadians(15.0), Math.toRadians(295.031)));
        }

        @Override
        public String backCam() {
            return "limelight-back";
        }

        @Override
        public Pose3d backCamPose() {
            return new Pose3d(
                    -0.2203704,
                    -0.3128518,
                    0.19939,
                    new Rotation3d(0.0, Math.toRadians(15.0), Math.toRadians(180.0)));
        }

        @Override
        public String leftCam() {
            return "limelight-left";
        }

        @Override
        public Pose3d leftCamPose() {
            return new Pose3d(
                    -0.3128518,
                    -0.2203704,
                    0.19939,
                    new Rotation3d(0.0, Math.toRadians(15.0), Math.toRadians(90.0)));
        }

        @Override
        public boolean disableVisionLogs() {
            return true;
        }
    }
}
