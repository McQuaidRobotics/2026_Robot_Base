// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package igknighters;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import igknighters.commands.teleop.TeleopSwerveWithDetune;
import igknighters.constants.DrivingSharedState;
import igknighters.constants.FirstBotConsts;
import igknighters.constants.RobotConsts;
import igknighters.constants.RobotIdentity;
import igknighters.constants.SecondBotConsts;
import igknighters.controllers.DriverController;
import igknighters.subsystems.LimeLightVision.LimeLightVision;
import igknighters.subsystems.LimeLightVision.LimeLightVision.pose_output;
import igknighters.subsystems.Luma.Luma;
import igknighters.subsystems.Subsystems;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.swerve.Swerve;
import igknighters.util.RobotPosePredError;
import igknighters.util.RobotPosePredictor;
import igknighters.util.TunableValues;
import igknighters.util.TunableValues.TunableDouble;
import igknighters.util.log.Log;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightSettings.ImuMode;
import limelight.networktables.Orientation3d;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * Robot base entry point.
 *
 * <p>This wires up the season independent robot: drivetrain, vision, LEDs, logging, pose estimation
 * and the auto/teleop lifecycle. It deliberately contains no game specific superstructure
 * mechanisms, commands or paths. A season repository adds those on top.
 */
public class Robot extends LoggedRobot {

    private Command m_autonomousCommand;

    public static RobotConsts consts;
    private AutoFactory autoFactory;
    public final AutoChooser autoChooser = new AutoChooser();
    public final AutoChooser testChooser = new AutoChooser();
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    public static RobotPosePredictor pose_pred;
    public static RobotPosePredError pose_pred_error = new RobotPosePredError();
    public static Orientation3d robotOrientation =
            new Orientation3d(
                    new Rotation3d(), new AngularVelocity3d(RPM.of(0), RPM.of(0), RPM.of(0)));
    private final DriverController driverController = new DriverController(0);

    public final Subsystems subsystems;

    private final boolean kUseLimelight = true;

    private Telemetry logger;
    TunableDouble detune = TunableValues.getDouble("Tunables/Detune", 1.0);
    TunableDouble targetingP = TunableValues.getDouble("Tunables/TargetingP", 0.07);
    TunableDouble targetingI = TunableValues.getDouble("Tunables/TargetingI", 0.00);
    TunableDouble targetingD = TunableValues.getDouble("Tunables/TargetingD", 0.00);

    public void setUpCommandLogging() {
        if (!Robot.consts.disableAllLogs()) {
            scheduler.onCommandInitialize(
                    command ->
                            Log.log(
                                    "Commands/Tracking/" + command.getName() + "/ Command Running",
                                    "TRUE"));

            scheduler.onCommandInitialize(
                    command ->
                            Log.log(
                                    "Commands/Tracking/"
                                            + command.getName()
                                            + "/ Command Interrupted",
                                    "FALSE"));

            scheduler.onCommandInterrupt(
                    command ->
                            Log.log(
                                    "Commands/Tracking/"
                                            + command.getName()
                                            + "/ Command Interrupted",
                                    "TRUE"));
            scheduler.onCommandFinish(
                    command ->
                            Log.log(
                                    "Commands/Tracking/" + command.getName() + "/ Command Running",
                                    "FALSE"));
            scheduler.onCommandFinish(
                    command ->
                            Log.log(
                                    "Commands/Tracking/"
                                            + command.getName()
                                            + "/ Command Interrupted",
                                    "FALSE"));
        }
    }

    public void publishCommandsAndSubystems(Subsystems subsystems) {
        SmartDashboard.putData(CommandScheduler.getInstance());
        for (var subsystem : subsystems.lockedResources) {
            SmartDashboard.putData("SubsystemCommands/" + subsystem.getName(), subsystem);
        }
    }

    public void setUpRobotConsts() {

        // THE IDS WILL BE WRONG SINCE SN IS WRONG WILL DEFAULT TO SECOND BOT
        if (RobotIdentity.isFirstBot()) {
            consts = new FirstBotConsts();
        } else if (RobotIdentity.isSecondBot()) {
            consts = new SecondBotConsts();
        } else if (Robot.isReal()) {
            throw new IllegalStateException(
                    "Unknown robot identity ENSURE SERIAL NUMBERS MATCH"); // only problem irl
        } else {
            consts = new FirstBotConsts(); // in sim with unknown sn we should pick something
        }
    }

    /**
     * Creates the auto factory and publishes the (empty) auto choosers.
     *
     * <p>The robot base ships no routines because auto paths are game specific. Register them in a
     * season repository with autoChooser.addRoutine(...).
     */
    public void setUpAutos(Subsystems subsystems) {
        autoFactory = subsystems.swerve.createAutoFactory();

        SmartDashboard.putData("AUTO CHOOSER", autoChooser);
        SmartDashboard.putData("TEST CHOOSER", testChooser);
    }

    /** Exposes the auto factory so a season repository can build routines against it. */
    public AutoFactory getAutoFactory() {
        return autoFactory;
    }

    public void setUpSwerve(Subsystems subsystems) {
        subsystems.swerve.setDefaultCommand(
                new TeleopSwerveWithDetune(subsystems.swerve, driverController, 1.0));

        logger = new Telemetry(subsystems.swerve.getMaxSpeedMetersPerSecond(), subsystems);
        subsystems.swerve.registerTelemetry(logger::telemeterize);
    }

    public void setUpAdvantageScope() {

        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata(
                "GitDirty",
                switch (BuildConstants.DIRTY) {
                    case 0 -> "All changes committed";
                    case 1 -> "Uncommitted changes";
                    default -> "Unknown";
                });
        try {
            Logger.recordMetadata(
                    "Hostname",
                    InetAddress.getLocalHost().getHostName().replaceAll("\\.local$", ""));
        } catch (UnknownHostException e) {
            Logger.recordMetadata("Hostname", "Unknown");
        }
        Logger.recordMetadata(
                "Platform",
                "%s %s (%s)"
                        .formatted(
                                System.getProperty("os.name").replace(" ", ""),
                                System.getProperty("os.version"),
                                System.getProperty("os.arch")));
        if (Robot.isReal()) {
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            Logger.addDataReceiver(new NT4Publisher());
        }

        // Set timing mode
        setUseTiming(true);

        // Start AdvantageKit logger
        Logger.start();
    }

    public Robot() {
        this(false);
    }

    public Robot(boolean isSwerveDisabled) {
        setUpRobotConsts();
        setUpAdvantageScope();
        setUpCommandLogging();
        subsystems =
                new Subsystems(
                        new Swerve(isSwerveDisabled),
                        new LimeLightVision(),
                        new Led(90, 2),
                        new Luma(true, "object-detection"));
        setUpSwerve(subsystems);
        pose_pred = new RobotPosePredictor(subsystems.swerve);
        publishCommandsAndSubystems(subsystems);
        setUpAutos(subsystems);
        bindDriverController();
    }

    public Supplier<Pose2d> poseSupplier() {
        return () -> subsystems.swerve.getState().Pose;
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        robotOrientation = subsystems.swerve.getRobotOrientation();
        pose_pred.setVelocitiesAndPose();
        pose_pred_error.logPose(subsystems.swerve.getState().Pose);

        if (kUseLimelight) {
            List<pose_output> vision_outputs = subsystems.vision.getRobotPoseFromVision();

            if (vision_outputs != null) {
                for (pose_output output : vision_outputs)
                    subsystems.swerve.addVisionMeasurement(
                            output.pose(),
                            output.time().in(Seconds)); // trusts vision rotation less. Needs tuning
                // increase the std devs to trust vision less
                if (!Robot.consts.limelightVision().disableVisionLogs()) {
                    Log.log("ROBOT/Subsystems/Vision/Null Pose", false);
                }
            } else {
                if (!Robot.consts.limelightVision().disableVisionLogs()) {
                    Log.log("ROBOT/Subsystems/Vision/Null Pose", true);
                }
            }
        }
    }

    public void bindDriverController() {
        driverController.bind(subsystems);
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearComposedCommands();
        subsystems.swerve.setDefaultCommand(
                new TeleopSwerveWithDetune(subsystems.swerve, driverController, detune.value()));
        DrivingSharedState.getInstance().setDetune(detune.value());
        DrivingSharedState.getInstance().setKP(targetingP.value());
        DrivingSharedState.getInstance().setKI(targetingI.value());
        DrivingSharedState.getInstance().setKD(targetingD.value());
        subsystems.vision.disableCameras();

        bindDriverController();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {
        subsystems.vision.enableCameras(ImuMode.ExternalImu);
    }

    @Override
    public void autonomousInit() {
        subsystems.vision.enableCameras(ImuMode.ExternalImu);
        Command autoCommand = autoChooser.selectedCommand();
        m_autonomousCommand = autoCommand;
        if (autoCommand != null) {
            scheduler.schedule(autoCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        subsystems.swerve.clearActiveTrajectory();
        scheduler.cancelAll();
    }

    @Override
    public void teleopInit() {
        subsystems.vision.enableCameras(ImuMode.ExternalImu);
        subsystems.swerve.clearActiveTrajectory();
        scheduler.cancelAll();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        Command autoCommand = testChooser.selectedCommand();
        if (autoCommand != null) {
            scheduler.schedule(autoCommand);
        }
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    public static boolean isRobotTest() {
        return RobotModeTriggers.test().getAsBoolean();
    }

    public static boolean isBlue() {
        Optional<Alliance> ally = DriverStation.getAlliance();

        if (ally.isPresent()) {
            return ally.get() == Alliance.Blue;
        } else {
            // Default to blue if alliance is unknown (e.g., in simulation without alliance set)
            // Log this so we know why things might be going to the blue side.
            if (!Robot.consts.disableAllLogs()) {
                Log.log("ROBOT/System/AllianceUnknown", true);
            }
            return true;
        }
    }
}
