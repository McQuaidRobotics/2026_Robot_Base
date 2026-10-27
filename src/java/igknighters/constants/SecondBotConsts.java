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
                    // simMount numbers: limelight-configs/<name>/pipeline_0.json rsf, rss, rsu,
                    // rspitch, rsyaw (pulled Oct 2026). Sim only, the real mount lives on the
                    // camera.
                    new CameraConfig("limelight").simMount(-0.2992, 0.2966, 0.335, 15, -135),
                    new CameraConfig("limelight-intake").simMount(0.0445, 0.3079, 0.2057, 15, -68),
                    new CameraConfig("limelight-back")
                            .throttleDisabled(0)
                            .simMount(-0.3129, -0.2237, 0.1994, 15, 180),
                    new CameraConfig("limelight-left").simMount(-0.2204, -0.3129, 0.1994, 15, 90));
        }

        @Override
        public boolean disableVisionLogs() {
            return false;
        }
    }
}
