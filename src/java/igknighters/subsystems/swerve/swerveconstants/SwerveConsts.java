package igknighters.subsystems.swerve.swerveconstants;

import edu.wpi.first.wpilibj.RobotController;
import igknighters.Robot;
import igknighters.util.log.Log;

public class SwerveConsts {

    String robotSerialNumber;

    public enum Robots {
        DEMO_BOT,
        FIRST_BOT,
        SECOND_BOT,
        UNKNOWN
    };

    private String DEMO_BOT_SERIAL_NUMBER = "TBD";
    private String FIRST_BOT_SERIAL_NUMBER = "032B4B20";
    private String SECOND_BOT_SERIAL_NUMBER = "03260ABB";

    public Robots getRobot() {
        robotSerialNumber = RobotController.getSerialNumber();
        if (!Robot.consts.disableAllLogs()) {
            Log.log("ROBOT/ROBOT_INFO/ROBOT SERIAL NUMBER", "Serial Number: " + robotSerialNumber);
        }
        if (robotSerialNumber.equals(DEMO_BOT_SERIAL_NUMBER)) {
            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/ROBOT_INFO/ROBOT TYPE", "DEMO_BOT");
            }
            return Robots.DEMO_BOT;
        } else if (robotSerialNumber.equals(FIRST_BOT_SERIAL_NUMBER)) {
            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/ROBOT_INFO/ROBOT TYPE", "FIRST_BOT");
            }
            return Robots.FIRST_BOT;
        } else if (robotSerialNumber.equals(SECOND_BOT_SERIAL_NUMBER)) {
            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/ROBOT_INFO/ROBOT TYPE", "SECOND_BOT");
            }
            return Robots.SECOND_BOT;
        } else {
            if (!Robot.consts.disableAllLogs()) {
                Log.log(
                        "ROBOT/ROBOT_INFO/ROBOT TYPE",
                        "UNKNOWN: first bot serial is: " + FIRST_BOT_SERIAL_NUMBER);
            }
            return Robots.UNKNOWN;
        }
    }

    public CommonSwerveConsts getSwerveConsts() {
        Robots robot = getRobot();
        if (robot.equals(Robots.DEMO_BOT)) {
            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/ROBOT_INFO/SWERVE CONSTS", "Using DemoBotConsts");
            }
            return new DemoBotConsts();
        } else if (robot.equals(Robots.FIRST_BOT)) {
            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/ROBOT_INFO/SWERVE CONSTS", "Using FirstBotSwerveConsts");
            }
            return new FirstBotSwerveConsts();
        } else if (robot.equals(Robots.SECOND_BOT)) {
            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/ROBOT_INFO/SWERVE CONSTS", "Using SecondBotSwerveConsts");
            }
            return new SecondBotSwerveConsts();
        } else {
            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/ROBOT_INFO/SWERVE CONSTS", "Using FirstBotSwerveConsts (default)");
            }
            return new FirstBotSwerveConsts();
        }
    }
}
