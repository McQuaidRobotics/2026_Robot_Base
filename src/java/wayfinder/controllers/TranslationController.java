package wayfinder.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayDeque;
import java.util.Queue;
import org.littletonrobotics.junction.Logger;
import wayfinder.controllers.Types.Constraints;
import wayfinder.controllers.Types.Controller;
import wayfinder.controllers.Types.State;
import wpilibExt.Velocity2d;

/**
 * A translation controller that implements feedback and various motion profiling schemes on the
 * distance to the target position and then extrapolates those into X and Y velocities.
 */
public abstract class TranslationController
        implements Controller<Translation2d, Velocity2d, Translation2d, Constraints> {

    @Override
    public abstract Velocity2d calculate(
            double period,
            Translation2d measurement,
            Velocity2d measurementVelo,
            Translation2d target,
            Constraints constraints);

    @Override
    public abstract void reset(
            Translation2d measurement, Velocity2d measurementVelo, Translation2d target);

    @Override
    public abstract boolean isDone(Translation2d measurement, Translation2d target);

    protected final double kP, kI, kD;

    private TranslationController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /** S-Curve (Jerk-Limited) Profile Controller with Rolling 100ms Window */
    private static final class SCurve extends TranslationController {
        private final boolean replanning;
        private final double positionTolerance;

        private double totalError;
        private State prevSetpoint = State.kZero;
        private double prevSetpointVel = 0.0;

        // Rolling window tracking
        private final Queue<Double> errorHistory = new ArrayDeque<>();
        private final double WINDOW_DURATION_SECONDS = 0.100; // 100 ms window

        public SCurve(
                double kP, double kI, double kD, boolean replanning, double positionTolerance) {
            super(kP, kI, kD);
            this.replanning = replanning;
            this.positionTolerance = positionTolerance;
        }

        @Override
        public boolean isDone(Translation2d measurement, Translation2d target) {
            return measurement.getDistance(target) < positionTolerance
                    && MathUtil.isNear(prevSetpoint.velocity(), 0.0, 0.01);
        }

        @Override
        public void reset(
                Translation2d measurement, Velocity2d measurementVelo, Translation2d target) {
            totalError = 0;
            errorHistory.clear(); // Clear history on reset

            final Rotation2d direction = target.minus(measurement).getAngle();
            final double distance = measurement.getDistance(target);

            prevSetpoint = new State(-distance, measurementVelo.speedInDirection(direction));
            prevSetpointVel = prevSetpoint.velocity();
        }

        @Override
        public Velocity2d calculate(
                double period,
                Translation2d measurement,
                Velocity2d measurementVelo,
                Translation2d target,
                Constraints constraints) {

            Logger.recordOutput("Wayfinder/TranslationController/Mode", "SCurve");

            if (isDone(measurement, target)) {
                return Velocity2d.kZero;
            }

            final double distance = measurement.getDistance(target);
            final Rotation2d direction = target.minus(measurement).getAngle();
            final double velo = measurementVelo.speedInDirection(direction);

            double currentAcceleration = (prevSetpoint.velocity() - prevSetpointVel) / period;

            State setpoint =
                    DynamicSCurveProfile.calculate(
                            period,
                            replanning ? -distance : prevSetpoint.position(),
                            replanning ? velo : prevSetpoint.velocity(),
                            currentAcceleration,
                            0.0,
                            0.0,
                            0.0,
                            constraints.maxVelocity(),
                            constraints.maxAcceleration(),
                            constraints.maxJerk());

            double positionError = distance + prevSetpoint.position();
            Logger.recordOutput("Wayfinder/TranslationController/SCurveError", positionError);

            // --- ROLLING 100MS WINDOW DERIVATIVE LOGIC ---
            errorHistory.add(positionError);

            // Determine how many frames fit in 100ms (e.g., 5 frames if period is 0.02s)
            int maxHistorySize = (int) Math.max(1, WINDOW_DURATION_SECONDS / period);
            while (errorHistory.size() > maxHistorySize) {
                errorHistory.poll(); // Evict oldest sample once we exceed 100ms
            }

            // Calculate derivative over the entire window span rather than just one loop
            double oldestError = errorHistory.peek();
            double totalWindowTime = errorHistory.size() * period;
            double errorDerivative = (positionError - oldestError) / totalWindowTime;
            // ----------------------------------------------

            if (kI > 0) {
                totalError +=
                        MathUtil.clamp(
                                positionError * period,
                                -constraints.maxAcceleration() * period / kI,
                                constraints.maxAcceleration() * period / kI);
            }

            prevSetpointVel = prevSetpoint.velocity();
            prevSetpoint = setpoint;

            double dirVelo =
                    (kP * positionError)
                            + (kI * totalError)
                            + (kD * errorDerivative) // Uses the smoothed derivative
                            + setpoint.velocity();

            dirVelo =
                    MathUtil.clamp(dirVelo, -constraints.maxVelocity(), constraints.maxVelocity());

            return new Velocity2d(dirVelo * direction.getCos(), dirVelo * direction.getSin());
        }
    }

    /** Trapezoidal Profile Controller (Original) */
    private static final class Profiled extends TranslationController {
        private final boolean replanning;
        private final double positionTolerance;

        private double prevError, totalError;
        private State prevSetpoint = State.kZero;

        public Profiled(
                double kP, double kI, double kD, boolean replanning, double positionTolerance) {
            super(kP, kI, kD);
            this.replanning = replanning;
            this.positionTolerance = positionTolerance;
        }

        @Override
        public boolean isDone(Translation2d measurement, Translation2d target) {
            return measurement.getDistance(target) < positionTolerance
                    && MathUtil.isNear(prevSetpoint.velocity(), 0.0, 0.01);
        }

        @Override
        public void reset(
                Translation2d measurement, Velocity2d measurementVelo, Translation2d target) {
            prevError = 0;
            totalError = 0;
            final Rotation2d direction = target.minus(measurement).getAngle();
            final double distance = measurement.getDistance(target);
            prevSetpoint = new State(-distance, measurementVelo.speedInDirection(direction));
        }

        @Override
        public Velocity2d calculate(
                double period,
                Translation2d measurement,
                Velocity2d measurementVelo,
                Translation2d target,
                Constraints constraints) {

            Logger.recordOutput("Wayfinder/TranslationController/Mode", "Profiled");

            if (isDone(measurement, target)) {
                return Velocity2d.kZero;
            }

            final double distance = measurement.getDistance(target);
            final Rotation2d direction = target.minus(measurement).getAngle();
            final double velo = measurementVelo.speedInDirection(direction);

            State setpoint =
                    DynamicTrapezoidProfile.calculate(
                            period,
                            replanning ? -distance : prevSetpoint.position(),
                            replanning ? velo : prevSetpoint.velocity(),
                            0.0,
                            0.0,
                            constraints.maxVelocity(),
                            constraints.maxAcceleration());

            double positionError = distance + prevSetpoint.position();

            Logger.recordOutput("Wayfinder/TranslationController/ProfiledError", positionError);

            double errorDerivative = (positionError - prevError) / period;
            if (kI > 0) {
                totalError +=
                        MathUtil.clamp(
                                positionError * period,
                                -constraints.maxAcceleration() * period / kI,
                                constraints.maxAcceleration() * period / kI);
            }
            prevError = positionError;
            prevSetpoint = setpoint;

            double dirVelo =
                    (kP * positionError)
                            + (kI * totalError)
                            + (kD * errorDerivative)
                            + setpoint.velocity();

            dirVelo =
                    MathUtil.clamp(dirVelo, -constraints.maxVelocity(), constraints.maxVelocity());
            return new Velocity2d(dirVelo * direction.getCos(), dirVelo * direction.getSin());
        }
    }

    /** Unprofiled Controller */
    private static final class UnProfiled extends TranslationController {
        private final double deadband;
        private double prevError, totalError;

        public UnProfiled(double kP, double kI, double kD, double deadband) {
            super(kP, kI, kD);
            this.deadband = deadband;
        }

        @Override
        public boolean isDone(Translation2d measurement, Translation2d target) {
            return measurement.getDistance(target) < deadband;
        }

        @Override
        public void reset(
                Translation2d measurement, Velocity2d measurementVelo, Translation2d target) {
            prevError = 0;
            totalError = 0;
        }

        @Override
        public Velocity2d calculate(
                double period,
                Translation2d measurement,
                Velocity2d measurementVelo,
                Translation2d target,
                Constraints constraints) {

            Logger.recordOutput("Wayfinder/TranslationController/Mode", "UnProfiled");

            if (isDone(measurement, target)) {
                return Velocity2d.kZero;
            }

            final double distance = measurement.getDistance(target);
            final Rotation2d direction = target.minus(measurement).getAngle();

            double positionError = distance;
            Logger.recordOutput("Wayfinder/TranslationController/UnProfiledError", positionError);
            double errorDerivative = (positionError - prevError) / period;
            if (kI > 0) {
                totalError += positionError * period;
            }
            prevError = positionError;

            double dirVelo = (kP * positionError) + (kI * totalError) + (kD * errorDerivative);
            dirVelo =
                    MathUtil.clamp(dirVelo, -constraints.maxVelocity(), constraints.maxVelocity());

            return new Velocity2d(dirVelo * direction.getCos(), dirVelo * direction.getSin());
        }
    }

    /** Unprofiled Controller */
    private static final class WPILIBCONTROLLER extends TranslationController {
        private final double deadband;
        private double prevError, totalError;
        private final PIDController profiledPIDController;

        public WPILIBCONTROLLER(double kP, double kI, double kD, double deadband) {
            super(kP, kI, kD);
            this.deadband = deadband;
            this.profiledPIDController = new PIDController(kP, kI, kD);
        }

        @Override
        public boolean isDone(Translation2d measurement, Translation2d target) {
            return measurement.getDistance(target) < deadband;
        }

        @Override
        public void reset(
                Translation2d measurement, Velocity2d measurementVelo, Translation2d target) {
            profiledPIDController.reset();
        }

        @Override
        public Velocity2d calculate(
                double period,
                Translation2d measurement,
                Velocity2d measurementVelo,
                Translation2d target,
                Constraints constraints) {

            Logger.recordOutput("Wayfinder/TranslationController/Mode", "UnProfiled");

            if (isDone(measurement, target)) {
                return Velocity2d.kZero;
            }

            final double distance = measurement.getDistance(target);
            final Rotation2d direction = target.minus(measurement).getAngle();

            double positionError = distance;
            Logger.recordOutput("Wayfinder/TranslationController/UnProfiledError", positionError);

            double pidOutput =
                    profiledPIDController.calculate(positionError, 0.0)
                            / distance; // normalize to -1 to 1

            double scaledOutput = pidOutput * 3;

            return new Velocity2d(
                    -scaledOutput * direction.getCos(), -scaledOutput * direction.getSin());
        }
    }

    // Static factory methods to expose the inner classes safely

    public static TranslationController scurve(
            double kP, double kI, double kD, boolean replanning, double positionTolerance) {
        return new SCurve(kP, kI, kD, replanning, positionTolerance);
    }

    public static TranslationController wpilib(double kP, double kI, double kD, double deadband) {
        return new WPILIBCONTROLLER(kP, kI, kD, deadband);
    }

    public static TranslationController profiled(
            double kP, double kI, double kD, boolean replanning, double positionTolerance) {
        return new Profiled(kP, kI, kD, replanning, positionTolerance);
    }

    public static TranslationController unprofiled(
            double kP, double kI, double kD, double deadband) {
        return new UnProfiled(kP, kI, kD, deadband);
    }
}
