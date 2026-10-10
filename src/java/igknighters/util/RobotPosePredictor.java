package igknighters.util;

import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import igknighters.Robot;
import igknighters.subsystems.swerve.Swerve;
import igknighters.util.Merging.PoseMerger;
import igknighters.util.Merging.SpeedsMerger;
import java.util.Optional;

/**
 * Estimates the robot's pose on the next loop iteration using an alpha-beta filter applied to a
 * rolling history of timestamped poses.
 *
 * <p>Alpha corrects the position estimate toward each new measurement. Beta corrects the velocity
 * estimate to reduce lag over time.
 */
public class RobotPosePredictor {
    Swerve swerve;

    // input is rotational speed

    /**
     * Returns the best possible pose prediction based on current state. Automatically switches to
     * Auto prediction if a trajectory is active.
     */
    public Pose2d getDynamicPredictedPose() {
        if (swerve.getActiveTrajectory() != null) {
            return getPredictedAutoPose(swerve);
        } else {
            return getPredictedPose();
        }
    }

    /**
     * Returns the best possible velocity prediction based on current state. Automatically switches
     * to Auto speeds if a trajectory is active.
     */
    public ChassisSpeeds getDynamicPredictedSpeeds() {
        if (swerve.getActiveTrajectory() != null) {
            return getPredictedAutoSpeeds(swerve);
        } else {
            return getPredictedVelos();
        }
    }

    private static final int HISTORY_SIZE = 10;

    public ChassisSpeeds[] veloHistory = new ChassisSpeeds[HISTORY_SIZE];
    public double[] timestampHistory = new double[HISTORY_SIZE];

    // time in seconds to look-ahead
    public static final double predTime = .02;

    Pose2d poseNow = new Pose2d();

    double[] accelerationsNow = new double[2];
    double[] rotVelos = new double[HISTORY_SIZE];

    /** Index of the next write slot in the circular buffers. */
    public int writeIndex = 0;

    /** Number of poses stored so far, capped at HISTORY_SIZE. */
    public int storedCount = 0;

    public RobotPosePredictor(Swerve swerve) {
        for (int i = 0; i < HISTORY_SIZE; i++) {
            veloHistory[i] = new ChassisSpeeds(0, 0, 0);
        }
        this.swerve = swerve;
    }

    /**
     * Records a new measured pose and updates the internal filter state. Should be called once per
     * loop iteration whenever a fresh pose is available.
     *
     * @param pose the latest measured robot pose
     */
    public void setVelocitiesAndPose() {

        double now = Timer.getFPGATimestamp();
        poseNow = swerve.getState().Pose;
        ChassisSpeeds chassisSpeeds = swerve.getFieldRelativeSpeeds();
        accelerationsNow[0] = swerve.getXAcceleration();
        accelerationsNow[1] = swerve.getYAcceleration();
        veloHistory[writeIndex].omegaRadiansPerSecond = swerve.getRotationalVelocity();
        // Write into circular buffer
        veloHistory[writeIndex].vxMetersPerSecond = chassisSpeeds.vxMetersPerSecond;
        veloHistory[writeIndex].vyMetersPerSecond = chassisSpeeds.vyMetersPerSecond;
        timestampHistory[writeIndex] = now;
        writeIndex = (writeIndex + 1) % HISTORY_SIZE;
        storedCount++;
        // if (veloHistory[HISTORY_SIZE - 1] != null) {
        //     Log.log("ROBOT/veloHistory", veloHistory);
        // }
    }

    public Pose2d getPredictedPose() {

        int latestIdx = indexOfMax(timestampHistory);
        int prevIdx = 0;
        if (latestIdx == 0) {
            prevIdx = HISTORY_SIZE - 1;
        } else {
            prevIdx = latestIdx - 1;
        }
        double dt = timestampHistory[latestIdx] - timestampHistory[prevIdx];

        if (dt <= 0.0) return poseNow;

        if (veloHistory[HISTORY_SIZE - 1] == null) {
            return poseNow;
        }

        double[] currentPose = poseToComponents(poseNow);
        double[] predicted = new double[3];

        // Predict next pose using predicted velocity (consider changing predicted velo to current
        // velo)
        double predictedRotAcc = getCalculatedRotAcceleration();
        predicted[0] =
                currentPose[0]
                        + veloHistory[latestIdx].vxMetersPerSecond * predTime
                        + 1 / 2 * accelerationsNow[0] * Math.pow(predTime, 2);
        predicted[1] =
                currentPose[1]
                        + veloHistory[latestIdx].vyMetersPerSecond * predTime
                        + 1 / 2 * accelerationsNow[1] * Math.pow(predTime, 2);
        // handle wrapping
        double predOmega =
                currentPose[2]
                        - veloHistory[latestIdx].omegaRadiansPerSecond * 0.08
                        - 1 / 2 * predictedRotAcc * Math.pow(0.08, 2);

        if (predOmega > Math.PI) {
            predicted[2] = predOmega - 2 * Math.PI;
        } else if (predOmega < -Math.PI) {
            predicted[2] = predOmega - 2 * Math.PI;
        } else {
            predicted[2] = predOmega;
        }

        Robot.pose_pred_error.findError(poseNow);

        return componentsToPose(predicted);
    }

    public ChassisSpeeds getPredictedVelos() {
        ChassisSpeeds predictedVelo = new ChassisSpeeds();
        int latestIdx = indexOfMax(timestampHistory);
        double predictedRotAcc = getCalculatedRotAcceleration();
        predictedVelo.vxMetersPerSecond =
                veloHistory[latestIdx].vxMetersPerSecond + accelerationsNow[0] * predTime;
        predictedVelo.vyMetersPerSecond =
                veloHistory[latestIdx].vyMetersPerSecond + accelerationsNow[1] * predTime;
        predictedVelo.omegaRadiansPerSecond =
                veloHistory[latestIdx].omegaRadiansPerSecond + predictedRotAcc * predTime;

        return predictedVelo;
    }

    public double getCalculatedRotAcceleration() {
        ChassisSpeeds predictedAcc = new ChassisSpeeds();
        int latestIdx = indexOfMax(timestampHistory);
        int prevIdx = 0;
        if (latestIdx == 0) {
            prevIdx = HISTORY_SIZE - 1;
        } else {
            prevIdx = latestIdx - 1;
        }
        double dt = timestampHistory[latestIdx] - timestampHistory[prevIdx];
        predictedAcc.omegaRadiansPerSecond =
                (veloHistory[latestIdx].omegaRadiansPerSecond
                                - veloHistory[prevIdx].omegaRadiansPerSecond)
                        / dt;
        return predictedAcc.omegaRadiansPerSecond;
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    /** Index of the first largest value (same as boxing + Collections.max + indexOf). */
    static int indexOfMax(double[] values) {
        int best = 0;
        for (int i = 1; i < values.length; i++) {
            if (values[i] > values[best]) best = i;
        }
        return best;
    }

    /** Index of the first smallest value (same as boxing + Collections.min + indexOf). */
    static int indexOfMin(double[] values) {
        int best = 0;
        for (int i = 1; i < values.length; i++) {
            if (values[i] < values[best]) best = i;
        }
        return best;
    }

    /** Decomposes a Pose2d into [x, y, rotation]. */
    private static double[] poseToComponents(Pose2d pose) {
        return new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
    }

    private static double[] poseToComponents(Pose3d pose) {
        return new double[] {pose.getX(), pose.getY(), pose.getZ(), pose.getRotation().getZ()};
    }

    /** Reconstructs a Pose2d from [x, y, roll, pitch]. */
    private static Pose2d componentsToPose(double[] c) {
        return new Pose2d(new Translation2d(c[0], c[1]), new Rotation2d(c[2]));
    }

    /**
     * Automatically pulls the active AutoTrajectory from Swerve and predicts where the robot should
     * be, adjusted by current physics.
     */
    public Pose2d getPredictedAutoPose(Swerve swerve) {
        AutoTrajectory activeTraj = swerve.getActiveTrajectory();

        // If we aren't running an auto path, just return the physics prediction
        if (activeTraj == null) {
            return getPredictedPose();
        }

        // Look ahead in the Choreo path
        double futureTime = swerve.getAutoTime() + predTime;

        // Extract the underlying trajectory data and sample it
        // Note: Choreo clamps the sample time internally if it exceeds the path length
        Trajectory<SwerveSample> trajectory = activeTraj.getRawTrajectory();
        Optional<SwerveSample> futureSampleOptional = trajectory.sampleAt(futureTime, true);
        Pose2d plannedFuturePose =
                futureSampleOptional.map(SwerveSample::getPose).orElseGet(() -> getPredictedPose());

        Pose2d physicsPrediction = getPredictedPose();
        // even though in theory the choreo is better in every way the real pose will always be more
        // important to care about we should just slightly modify real pose by infusing with choreo

        return PoseMerger.trustedMerge(physicsPrediction, plannedFuturePose);
    }

    /** Extracts the expected speeds from the active AutoTrajectory. */
    public ChassisSpeeds getPredictedAutoSpeeds(Swerve swerve) {
        AutoTrajectory activeTraj = swerve.getActiveTrajectory();
        if (activeTraj == null) return getPredictedVelos();

        Trajectory<SwerveSample> trajectory = activeTraj.getRawTrajectory();
        Optional<SwerveSample> futureSampleOptional =
                trajectory.sampleAt(swerve.getAutoTime() + predTime, true);

        ChassisSpeeds predictedSpeeds = getPredictedVelos();
        if (futureSampleOptional.isPresent()) {
            return SpeedsMerger.trustedMerge(
                    predictedSpeeds, futureSampleOptional.get().getChassisSpeeds());
        } else {
            return getPredictedVelos();
        }
    }
}
