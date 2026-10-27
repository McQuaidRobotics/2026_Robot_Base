package igknighters.subsystems.swerve;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
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

    /**
     * Sim only: where the robot really is. Driven by the simulated wheels and never by vision, so
     * the vision sim can be fed truth instead of our own estimate (which made a feedback loop).
     */
    private Pose2d simTruthPose = Pose2d.kZero;

    private double lastSimTruthTime = -1;

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
            if (Robot.isSimulation()) {
                updateSimTruthPose();
            }
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
        simTruthPose = pose;
    }

    /** Correct the heading only (keeps x/y). The sim truth pose is not touched: it is truth. */
    public void resetRotation(Rotation2d rotation) {
        if (!isSwerveDisabled) {
            drivetrain.resetRotation(rotation);
        }
    }

    /**
     * Sim only: make the robot believe its heading is {@code errorDeg} off from the truth (like
     * booting facing the wrong way), without moving the truth. For testing the vision heading fix.
     */
    public void simBreakHeading(double errorDeg) {
        if (Robot.isSimulation()) {
            resetRotation(simTruthPose.getRotation().plus(Rotation2d.fromDegrees(errorDeg)));
        }
    }

    /** Sim only: the robot's true pose, for feeding camera simulators. */
    public Pose2d getSimTruthPose() {
        return simTruthPose;
    }

    private void updateSimTruthPose() {
        double now = Timer.getFPGATimestamp();
        if (lastSimTruthTime >= 0) {
            double dt = now - lastSimTruthTime;
            ChassisSpeeds v = getState().Speeds; // robot relative, from the simulated modules
            simTruthPose =
                    simTruthPose.exp(
                            new Twist2d(
                                    v.vxMetersPerSecond * dt,
                                    v.vyMetersPerSecond * dt,
                                    v.omegaRadiansPerSecond * dt));
        }
        lastSimTruthTime = now;
        Logger.recordOutput("Sim/TruthPose", simTruthPose);
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
                    visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
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
