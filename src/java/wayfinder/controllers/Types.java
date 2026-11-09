package wayfinder.controllers;

import edu.wpi.first.math.geometry.Pose2d;
import wpilibExt.Speeds.FieldSpeeds;

public class Types {
    public record State(double position, double velocity) {
        public static final State kZero = new State(0, 0);
    }

    // FIX: Renamed the 4th generic parameter from 'Constraints' to 'ConstraintType'
    public interface Controller<Measurement, MeasurementRate, Target, ConstraintType> {
        MeasurementRate calculate(
                double period,
                Measurement measurement,
                MeasurementRate measurementRate,
                Target target,
                ConstraintType constraints); // Uses the generic type parameter

        void reset(Measurement measurement, MeasurementRate measurementRate, Target target);

        boolean isDone(Measurement measurement, Target target);
    }

    // This is the concrete record that was being hidden!
    public record Constraints(double maxVelocity, double maxAcceleration, double maxJerk) {}

    public record ChassisConstraints(Constraints translation, Constraints rotation) {
        public static final ChassisConstraints kZero =
                new ChassisConstraints(new Constraints(0, 0, 0), new Constraints(0, 0, 0));
    }

    public interface PathFollower {
        FieldSpeeds calculate(double dt, Pose2d currentPose, Pose2d targetPose);

        boolean isDone(Pose2d currentPose, Pose2d targetPose);

        void reset(Pose2d currentPose, Pose2d targetPose);
    }
}
