package igknighters.constants;

import igknighters.subsystems.LimeLightVision.CameraConfig;
import igknighters.subsystems.swerve.swerveconstants.CommonSwerveConsts;
import igknighters.subsystems.swerve.swerveconstants.SwerveConsts;
import java.util.List;

public class SecondBotConsts extends RobotConsts {

    public static final boolean disableAllLogs = true;

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
        return new SecondBotLimelightVisionConsts();
    }

    public static class SecondBotLimelightVisionConsts implements kLimelightVisionConsts {
        @Override
        public List<CameraConfig> cameras() {
            return List.of(
                    new CameraConfig("limelight").throttleDisabled(0),
                    new CameraConfig("limelight-intake").throttleDisabled(0),
                    new CameraConfig("limelight-back").throttleDisabled(0),
                    new CameraConfig("limelight-left").throttleDisabled(0));
        }

        @Override
        public boolean disableVisionLogs() {
            return false;
        }
    }
}
