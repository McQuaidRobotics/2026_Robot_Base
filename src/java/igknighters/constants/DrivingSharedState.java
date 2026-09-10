package igknighters.constants;

import edu.wpi.first.math.MathUtil;

public class DrivingSharedState {

    private DrivingSharedState() {}

    private static class SingletonHelper {
        private static final DrivingSharedState INSTANCE = new DrivingSharedState();
    }

    public static DrivingSharedState getInstance() {
        return SingletonHelper.INSTANCE;
    }

    public double detune = 1.0;
    public double kP = 0.07;
    public double kI = 0.00;
    public double kD = 0.00;

    public void setKP(double p) {
        this.kP = p;
    }

    public void setKI(double i) {
        this.kI = i;
    }

    public void setKD(double d) {
        this.kD = d;
    }

    public void setDetune(double detune) {
        this.detune = MathUtil.clamp(detune, 0, 1.0);
    }
}
