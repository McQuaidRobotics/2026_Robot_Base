package igknighters.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import igknighters.Robot;
import igknighters.util.log.Log;

public class RobotPosePredError {
    Pose2d[] predictionHistory = new Pose2d[2];
    double[] timestampHistory = new double[2];
    int writeIndex = 0;

    public RobotPosePredError() {
        for (int i = 0; i < 2; i++) {
            predictionHistory[i] = new Pose2d(0, 0, new Rotation2d());
        }
    }

    public void logPose(Pose2d currentPose) {
        predictionHistory[writeIndex] = Robot.pose_pred.getDynamicPredictedPose();
        timestampHistory[writeIndex] = Timer.getFPGATimestamp();
        if (writeIndex != 1) {
            writeIndex++;
        } else {
            writeIndex = 0;
        }
        // Log.log("ROBOT/tsdifference", timestampHistory[0] - timestampHistory[1]);
    }

    public double[] findError(Pose2d actualPose) {
        int latestIdx = RobotPosePredictor.indexOfMin(timestampHistory);
        double[] errors = new double[3];
        errors[0] =
                Math.abs(
                        (actualPose.getX() - predictionHistory[latestIdx].getX())
                                / actualPose.getX());
        errors[1] =
                Math.abs(
                        (actualPose.getY() - predictionHistory[latestIdx].getY())
                                / actualPose.getY());
        errors[2] =
                Math.abs(
                        (actualPose.getRotation().getDegrees()
                                        - predictionHistory[latestIdx].getRotation().getDegrees())
                                / actualPose.getRotation().getDegrees());
        Log.log("ROBOT/pose_pred_error", errors);
        return errors;
    }
}
