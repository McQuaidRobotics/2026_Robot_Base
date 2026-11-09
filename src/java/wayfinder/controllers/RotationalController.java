package wayfinder.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import wayfinder.controllers.Types.Constraints;
import wayfinder.controllers.Types.Controller;
import wayfinder.controllers.Types.State;

public abstract class RotationalController
        implements Controller<Rotation2d, Double, Rotation2d, Constraints> {

    protected static boolean withinTolerance(
            Rotation2d lhs, Rotation2d rhs, double toleranceRadians) {
        if (Math.abs(toleranceRadians) > Math.PI) {
            return true;
        }
        double dot = lhs.getCos() * rhs.getCos() + lhs.getSin() * rhs.getSin();
        return dot > Math.cos(toleranceRadians);
    }

    protected final double kP, kD;

    private RotationalController(double kP, double kD) {
        this.kP = kP;
        this.kD = kD;
    }

    @Override
    public abstract Double calculate(
            double period,
            Rotation2d measurement,
            Double measurementVelo,
            Rotation2d target,
            Constraints constraints);

    @Override
    public abstract void reset(Rotation2d measurement, Double measurementVelo, Rotation2d target);

    @Override
    public abstract boolean isDone(Rotation2d measurement, Rotation2d target);

    private static class Profiled extends RotationalController {
        private final boolean replanning;
        private final double positionTolerance; // Added for robust physical checking

        private double prevError = 0;
        private State prevSetpoint = State.kZero;

        public Profiled(double kP, double kD, boolean replanning, double positionTolerance) {
            super(kP, kD);
            this.replanning = replanning;
            this.positionTolerance = positionTolerance;
        }

        @Override
        public boolean isDone(Rotation2d measurement, Rotation2d target) {
            // FIX: Leverages your geometric helper to avoid wrapping bugs
            // AND ensures the physical robot is actually pointing the right way.
            return withinTolerance(measurement, target, positionTolerance)
                    && MathUtil.isNear(prevSetpoint.velocity(), 0.0, 0.01);
        }

        @Override
        public Double calculate(
                double period,
                Rotation2d measurementGeom,
                Double measurementVelo,
                Rotation2d targetGeom,
                Constraints constraints) {

            if (isDone(measurementGeom, targetGeom)) {
                return 0.0;
            }

            double measurement = measurementGeom.getRadians();
            double target = targetGeom.getRadians();

            measurement = MathUtil.angleModulus(measurement);
            target = MathUtil.angleModulus(target);

            // Ensure that the setpoint is always the shortest path to the target
            target = MathUtil.angleModulus(target - measurement) + measurement;
            double wrappedSetpoint =
                    MathUtil.angleModulus(prevSetpoint.position() - measurement) + measurement;
            if (Math.abs(wrappedSetpoint - prevSetpoint.position()) > 0.001) {
                prevSetpoint = new State(wrappedSetpoint, prevSetpoint.velocity());
            }

            // Calculate intermediate setpoint based on constraints
            State setpoint =
                    DynamicTrapezoidProfile.calculate(
                            period,
                            replanning ? measurement : prevSetpoint.position(),
                            replanning ? measurementVelo : prevSetpoint.velocity(),
                            target,
                            0.0,
                            constraints.maxVelocity(),
                            constraints.maxAcceleration());

            // Calculate the error and derivative of the error
            double positionError = MathUtil.angleModulus(prevSetpoint.position() - measurement);
            double errorOverTime = (positionError - prevError) / period;
            prevError = positionError;

            prevSetpoint = setpoint;

            // Add feedback of the PD controller to the feedforward velocity
            double ret = (kP * positionError) + (kD * errorOverTime) + setpoint.velocity();

            return MathUtil.clamp(ret, -constraints.maxVelocity(), constraints.maxVelocity());
        }

        @Override
        public void reset(Rotation2d measurement, Double measurementVelo, Rotation2d target) {
            prevError = 0.0;
            prevSetpoint = new State(measurement.getRadians(), measurementVelo);
        }
    }

    private static class UnProfiled extends RotationalController {
        private final double deadband;
        private double prevError;

        public UnProfiled(double kP, double kD, double deadband) {
            super(kP, kD);
            this.deadband = deadband;
        }

        @Override
        public boolean isDone(Rotation2d measurement, Rotation2d target) {
            return withinTolerance(measurement, target, deadband);
        }

        @Override
        public void reset(Rotation2d measurement, Double measurementVelo, Rotation2d target) {
            prevError = 0;
        }

        @Override
        public Double calculate(
                double period,
                Rotation2d measurementGeom,
                Double measurementVelo,
                Rotation2d targetGeom,
                Constraints constraints) {

            if (isDone(measurementGeom, targetGeom)) {
                return 0.0;
            }
            double measurement = measurementGeom.getRadians();
            double target = targetGeom.getRadians();

            double positionError = MathUtil.angleModulus(target - measurement);
            double errorOverTime = (positionError - prevError) / period;
            prevError = positionError;

            double ret = (kP * positionError) + (kD * errorOverTime);

            // FIX: Clamp output to prevent absolute insanity on large unprofiled step-changes
            return MathUtil.clamp(ret, -constraints.maxVelocity(), constraints.maxVelocity());
        }
    }

    public static RotationalController profiled(
            double kP, double kD, boolean replanning, double positionTolerance) {
        return new Profiled(kP, kD, replanning, positionTolerance);
    }

    public static RotationalController unprofiled(double kP, double kD, double deadband) {
        return new UnProfiled(kP, kD, deadband);
    }
}
