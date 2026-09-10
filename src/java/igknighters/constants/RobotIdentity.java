package igknighters.constants;

import edu.wpi.first.wpilibj.RobotController;

public class RobotIdentity {

    public enum Robots {
        FIRST_BOT,
        DEMO_BOT,
        SECOND_BOT,
        UNKNOWN
    }

    private static final String FIRST_BOT_SERIAL_NUMBER = "032B4B20";
    private static final String SECOND_BOT_SERIAL_NUMBER = "03260ABB"; // To be filled in later

    private static Robots robot = null;

    public static Robots getRobot() {
        if (robot == null) {
            String serialNumber = RobotController.getSerialNumber();
            if (serialNumber.equals(FIRST_BOT_SERIAL_NUMBER)) {
                robot = Robots.FIRST_BOT;
            } else if (serialNumber.equals(SECOND_BOT_SERIAL_NUMBER)) {
                robot = Robots.SECOND_BOT;
            } else if (serialNumber.equals("TBD")) { // Placeholder for Demo Bot if different
                robot = Robots.FIRST_BOT;
            } else {
                robot = Robots.FIRST_BOT; // will default to FIRST_BOT
            }
        }
        return robot;
    }

    public static boolean isFirstBot() {
        return getRobot() == Robots.FIRST_BOT;
    }

    public static boolean isSecondBot() {
        return getRobot() == Robots.SECOND_BOT;
    }
}
