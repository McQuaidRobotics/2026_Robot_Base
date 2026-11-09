package wayfinder.setpointGenerator;

import edu.wpi.first.math.geometry.Rotation2d;
import wpilibExt.Speeds.FieldSpeeds;
import wpilibExt.Speeds.RobotSpeeds;

public record SwerveSetpoint(
        RobotSpeeds speeds, AdvancedSwerveModuleState[] moduleStates, Rotation2d heading) {
    public static SwerveSetpoint zeroed() {
        return new SwerveSetpoint(
                RobotSpeeds.kZero,
                new AdvancedSwerveModuleState[] {
                    new AdvancedSwerveModuleState(0, Rotation2d.kZero, 0),
                    new AdvancedSwerveModuleState(0, Rotation2d.kZero, 0),
                    new AdvancedSwerveModuleState(0, Rotation2d.kZero, 0),
                    new AdvancedSwerveModuleState(0, Rotation2d.kZero, 0)
                },
                Rotation2d.kZero);
    }

    public final FieldSpeeds fieldSpeeds() {
        return speeds.asFieldRelative(heading);
    }
}
