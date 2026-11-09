package wayfinder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import wayfinder.controllers.PositionalController;
import wayfinder.controllers.Types.ChassisConstraints;
import wayfinder.repulsorField.Obstacle;
import wayfinder.repulsorField.RepulsorFieldPlanner;
import wayfinder.setpointGenerator.SwerveSetpoint;
import wayfinder.setpointGenerator.SwerveSetpointGenerator;
import wpilibExt.Speeds;
import wpilibExt.Speeds.FieldSpeeds;

/**
 * Centered management class for the Wayfinder library. Use this to register obstacles, calculate
 * paths, and toggle debug visualizations.
 */
public class WayfinderManager {
    private static final List<Obstacle> registeredObstacles = new ArrayList<>();
    private static RepulsorFieldPlanner planner;
    private static SwerveSetpointGenerator generator;

    private static boolean arrowsEnabled = false;
    private static boolean heatMapEnabled = false;
    private static SwerveSetpoint lastSetpoint = SwerveSetpoint.zeroed();
    private static PositionalController controller;

    /**
     * Configure the Wayfinder system with robot-specific controllers and kinematics. This should be
     * called once during robot initialization.
     */
    public static void setup(
            PositionalController positionalController, SwerveSetpointGenerator setpointGenerator) {
        controller = positionalController;
        generator = setpointGenerator;
        refreshPlanner();
    }

    /** Add an obstacle to the field. Rebuilds the planner automatically. */
    public static void addObstacle(Obstacle obs) {
        registeredObstacles.add(obs);
        refreshPlanner();
    }

    public static void addObstacles(Obstacle[] obs) {
        for (Obstacle o : obs) {
            registeredObstacles.add(o);
        }
        refreshPlanner();
    }

    /** Remove all obstacles from the field. */
    public static void clearObstacles() {
        registeredObstacles.clear();
        refreshPlanner();
    }

    /** Enable or disable the "Vector Field" arrows in AdvantageScope. */
    public static void setArrowsEnabled(boolean enabled) {
        arrowsEnabled = enabled;
        if (!enabled) {
            Logger.recordOutput("Pose/Wayfinder/Arrows", new Pose2d[0]);
        }
    }

    public static void setHeatMapEnabled(boolean enabled) {
        heatMapEnabled = enabled;
        if (!enabled) {
            Logger.recordOutput("Pose/Wayfinder/HeatMap", new Pose2d[0]);
        }
    }

    /**
     * Main calculation loop. Call this periodically to get the next swerve setpoint.
     *
     * @param dt Loop period (e.g. 0.02s)
     * @param currentPose Robot's current estimated pose
     * @param currentSpeeds Robot's current measured speeds. These should be robot relative
     * @param targetPose Desired target pose
     * @param constraints Motion constraints for this movement
     * @return A SwerveSetpoint optimized for physics and obstacle avoidance
     */
    public static SwerveSetpoint calculate(
            double dt,
            Pose2d currentPose,
            ChassisSpeeds chassisSpeeds,
            Pose2d targetPose,
            ChassisConstraints constraints) {

        if (planner == null || generator == null) return lastSetpoint;

        Speeds currentSpeeds = Speeds.fromRobotRelative(chassisSpeeds);

        // 1. Determine safe directional field speeds (avoiding obstacles)
        FieldSpeeds desiredFieldSpeeds =
                planner.calculate(dt, currentPose, currentSpeeds, targetPose, constraints);

        // 2. Constrain speeds to physical limits (Torque, Friction, Kinematics)
        lastSetpoint =
                generator.generateSetpoint(
                        lastSetpoint,
                        currentPose.getRotation(),
                        Speeds.fromFieldRelative(
                                desiredFieldSpeeds.vx(),
                                desiredFieldSpeeds.vy(),
                                desiredFieldSpeeds.omega()),
                        Optional.of(constraints),
                        dt);

        // 3. Debug visualization
        if (arrowsEnabled) {
            Logger.recordOutput(
                    "Pose/Wayfinder/Arrows",
                    planner.getArrows(
                            targetPose.getTranslation(),
                            currentPose.getTranslation(),
                            20,
                            10,
                            false));
        }

        if (heatMapEnabled) {
            Logger.recordOutput(
                    "Pose/Wayfinder/HeatMap",
                    planner.getHeatMap(
                            targetPose.getTranslation(),
                            currentPose.getTranslation(),
                            20,
                            10,
                            0.3,
                            false));
        }

        return lastSetpoint;
    }

    /** Reset the internal state of the controllers. Call when starting a new movement. */
    public static void reset(Pose2d currentPose, FieldSpeeds currentSpeeds, Pose2d targetPose) {
        if (planner != null) {
            planner.reset(currentPose, currentSpeeds, targetPose);
        }
        lastSetpoint = SwerveSetpoint.zeroed();
    }

    private static void refreshPlanner() {
        if (controller != null) {
            planner =
                    new RepulsorFieldPlanner(
                            controller, registeredObstacles.toArray(new Obstacle[0]));
        }
    }
}
