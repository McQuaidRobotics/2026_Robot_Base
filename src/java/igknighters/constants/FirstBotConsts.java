package igknighters.constants;

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
        public String frontCam() {
            return "limelight-intake";
        }

        @Override
        public String backCam() {
            return "limelight-back";
        }

        @Override
        public String rightCam() {
            return "limelight-left";
        }

        @Override
        public boolean disableVisionLogs() {
            return true;
        }
    }
}
