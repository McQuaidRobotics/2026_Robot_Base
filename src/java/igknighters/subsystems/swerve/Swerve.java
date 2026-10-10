package igknighters.subsystems.swerve;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import igknighters.Robot;
import igknighters.constants.Conv;
import igknighters.subsystems.swerve.swerveconstants.CommonSwerveConsts;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
    CommandSwerveDrivetrain drivetrain;
    CommonSwerveConsts commonSwerveConsts;
    boolean isSwerveDisabled = false;
    DummySwerve dummySwerve = new DummySwerve();
    // Cached so each loop skips Phoenix's synchronized signal lookup
    private StatusSignal<?> accelX, accelY, angularVelocityZ;

    public Swerve() {
        this(false);
    }

    public Swerve(boolean isSwerveDisabled) {
        this.isSwerveDisabled = isSwerveDisabled;
        if (!isSwerveDisabled) {
            drivetrain = Robot.consts.swerve().getCommonSwerveConsts().createDrivetrain(this);
            commonSwerveConsts = Robot.consts.swerve().getCommonSwerveConsts();
            accelX = drivetrain.getPigeon2().getAccelerationX();
            accelY = drivetrain.getPigeon2().getAccelerationY();
            angularVelocityZ = drivetrain.getPigeon2().getAngularVelocityZDevice();
        }
    }

    @Override
    public void periodic() {
        if (!isSwerveDisabled) {
            drivetrain.periodic();
            if (Robot.isRobotTest()) {
                Logger.recordOutput(
                        "ROBOT/TEST/SWERVE/CURRENT ROTATION DEGREES",
                        drivetrain.getPigeon2().getYaw().getValueAsDouble());
            }
        }
    }

    public void followPath(SwerveSample sample) {
        if (!isSwerveDisabled) {
            drivetrain.followPath(sample);
        }
    }

    public void resetPose(Pose2d pose) {
        if (!isSwerveDisabled) {
            drivetrain.resetPose(pose);
        }
    }

    public AutoFactory createAutoFactory() {
        if (!isSwerveDisabled) {
            return drivetrain.createAutoFactory();
        } else {
            return new AutoFactory(
                    () -> new Pose2d(), this::resetPose, this::followPath, true, this);
        }
    }

    public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> logger) {
        if (!isSwerveDisabled) {
            return drivetrain.createAutoFactory(logger);
        } else {
            return new AutoFactory(
                    () -> new Pose2d(), this::resetPose, this::followPath, true, this, logger);
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        if (!isSwerveDisabled) {
            return drivetrain.applyRequest(requestSupplier);
        } else {
            return dummySwerve.doNothing();
        }
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        if (!isSwerveDisabled) {
            return drivetrain.sysIdQuasistatic(direction);
        } else {
            return dummySwerve.doNothing();
        }
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        if (!isSwerveDisabled) {
            return drivetrain.sysIdDynamic(direction);
        } else {
            return dummySwerve.doNothing();
        }
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
        if (!isSwerveDisabled) {
            drivetrain.addVisionMeasurement(visionPose, timestamp);
        }
    }

    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        if (!isSwerveDisabled) {
            drivetrain.addVisionMeasurement(
                    visionRobotPoseMeters,
                    Utils.fpgaToCurrentTime(timestampSeconds),
                    visionMeasurementStdDevs);
        }
    }

    public SwerveDriveState getState() {
        if (!isSwerveDisabled) {
            return drivetrain.getState();
        } else {
            return new SwerveDriveState();
        }
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        if (!isSwerveDisabled) {
            var state = drivetrain.getState();
            return ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation());
        } else {
            return new ChassisSpeeds();
        }
    }

    public void registerTelemetry(Consumer<SwerveDriveState> telemetryFunction) {
        if (!isSwerveDisabled) {
            drivetrain.registerTelemetry(telemetryFunction);
        }
    }

    public void setControl(SwerveRequest request) {
        if (!isSwerveDisabled) {
            drivetrain.setControl(request);
        }
    }

    public double getMaxSpeedMetersPerSecond() {
        if (!isSwerveDisabled) {
            return commonSwerveConsts.getMaxSpeedMetersPerSecond();
        } else {
            return 0.0;
        }
    }

    public double getXAcceleration() {
        if (!isSwerveDisabled) {
            return accelX.refresh().getValueAsDouble();
        } else {
            return 0.0;
        }
    }

    public double getYAcceleration() {
        if (!isSwerveDisabled) {
            return accelY.refresh().getValueAsDouble();
        } else {
            return 0.0;
        }
    }

    public double getRotationalVelocity() {
        if (!isSwerveDisabled) {
            return angularVelocityZ.refresh().getValueAsDouble() * Conv.DEGREES_TO_RADIANS;
        } else {
            return 0.0;
        }
    }

    private AutoTrajectory activeTrajectory = null;
    private final Timer autoTimer = new Timer();

    public void setActiveTrajectory(AutoTrajectory trajectory) {
        this.activeTrajectory = trajectory;
        autoTimer.restart(); // Reset and start the timer
    }

    public void clearActiveTrajectory() {
        this.activeTrajectory = null;
        autoTimer.stop();
    }

    public AutoTrajectory getActiveTrajectory() {
        return activeTrajectory;
    }

    public double getAutoTime() {
        return autoTimer.get();
    }
}
