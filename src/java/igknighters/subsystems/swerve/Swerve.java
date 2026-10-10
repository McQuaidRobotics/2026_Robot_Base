package igknighters.subsystems.swerve;

import static edu.wpi.first.units.Units.RPM;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
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
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.Orientation3d;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
    CommandSwerveDrivetrain drivetrain;
    CommonSwerveConsts commonSwerveConsts;
    boolean isSwerveDisabled = false;
    DummySwerve dummySwerve = new DummySwerve();

    public Swerve() {
        this(false);
    }

    public Swerve(boolean isSwerveDisabled) {
        this.isSwerveDisabled = isSwerveDisabled;
        if (!isSwerveDisabled) {
            drivetrain = Robot.consts.swerve().getCommonSwerveConsts().createDrivetrain(this);
            commonSwerveConsts = Robot.consts.swerve().getCommonSwerveConsts();
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

    public AngularVelocity3d getAngularVelocity() {
        if (!isSwerveDisabled) {
            // NOTE: IT IS UNKOWN AS TO WHETHER LL WANTS THE WORLD PERSPECTIVE OR ROBOT
            // ALSO I DO NOT KNOW IF I HAVE TO HANDLE THE WEIRD MOUNTING MANUALLY
            return new AngularVelocity3d(
                    drivetrain.getPigeon2().getAngularVelocityXDevice().getValue(),
                    drivetrain.getPigeon2().getAngularVelocityYDevice().getValue(),
                    drivetrain.getPigeon2().getAngularVelocityZDevice().getValue());
        } else {
            return new AngularVelocity3d(RPM.of(0), RPM.of(0), RPM.of(0));
        }
    }

    public Orientation3d getRobotOrientation() {
        // need to test irl to see how the rotation is effected by mounting
        if (!isSwerveDisabled) {
            return new Orientation3d(drivetrain.getPigeon2().getRotation3d(), getAngularVelocity());
        } else {
            return new Orientation3d(new Rotation3d(), getAngularVelocity());
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
            return drivetrain.getPigeon2().getAccelerationX().getValueAsDouble();
        } else {
            return 0.0;
        }
    }

    public double getYAcceleration() {
        if (!isSwerveDisabled) {
            return drivetrain.getPigeon2().getAccelerationY().getValueAsDouble();
        } else {
            return 0.0;
        }
    }

    public double getRotationalVelocity() {
        if (!isSwerveDisabled) {
            return drivetrain.getPigeon2().getAngularVelocityZDevice().getValueAsDouble()
                    * Conv.DEGREES_TO_RADIANS;
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
