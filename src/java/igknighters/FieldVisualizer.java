package igknighters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import igknighters.util.TunableValues;
import igknighters.util.TunableValues.TunableBoolean;
import java.util.List;

public class FieldVisualizer {
    private final Field2d field;

    private FieldVisualizer() {
        field = new Field2d();
        // The single entry point to SmartDashboard for field widget visuals
        SmartDashboard.putData("Field", field);
    }

    public Field2d getField() {
        return field;
    }

    private static class SingletonHelper {
        private static final FieldVisualizer INSTANCE = new FieldVisualizer();
    }

    public static FieldVisualizer getInstance() {
        return SingletonHelper.INSTANCE;
    }

    private final TunableBoolean shouldShowDetectedObjects =
            TunableValues.getBoolean("FieldVisualizer/ShowDetectedObjects", true);
    private final TunableBoolean shouldShowDrivingTarget =
            TunableValues.getBoolean("FieldVisualizer/ShowDrivingTarget", true);

    public void updatePredictedPose(Pose2d pred_pose) {
        if (pred_pose == null) {
            field.getObject("PREDICTED_POSE").setPose(new Pose2d());
            return;
        }
        field.getObject("PREDICTED_POSE").setPose(pred_pose);
    }

    /**
     * Updates the driving target pose on the field.
     *
     * @param target The pose of the driving target, or null to clear.
     */
    public void updateDrivingTarget(Pose2d target) {
        if (target == null || !shouldShowDrivingTarget.value()) {
            field.getObject("DRIVING_TARGET").setPose(new Pose2d());
            return;
        }
        field.getObject("DRIVING_TARGET").setPose(target);
    }

    /**
     * Updates the list of detected objects on the field.
     *
     * @param objects A list of poses for detected objects, or null/empty to clear.
     */
    public void updateDetectedObjects(List<Pose2d> objects) {
        if (objects == null || objects.isEmpty() || !shouldShowDetectedObjects.value()) {
            field.getObject("DETECTED_OBJECTS").setPoses(List.of());
            return;
        }
        field.getObject("DETECTED_OBJECTS").setPoses(objects);
    }

    public void addVelocityVector(Pose2d velocityVector) {
        if (velocityVector != null) {
            field.getObject("ACTUAL_VELOCITY").setPose(velocityVector);
        } else {
            field.getObject("ACTUAL_VELOCITY").setPose(new Pose2d());
        }
    }

    public void addPredictedVelocityVector(Pose2d predictedVelocityVector) {
        if (predictedVelocityVector != null) {
            field.getObject("PREDICTED_VELOCITY").setPose(predictedVelocityVector);
        } else {
            field.getObject("PREDICTED_VELOCITY").setPose(new Pose2d());
        }
    }

    /**
     * Updates all vision-related targets on the field in one call.
     *
     * @param detectedObjects List of detected object poses.
     * @param drivingTarget Driving target pose.
     */
    public void updateVisionTargets(List<Pose2d> detectedObjects, Pose2d drivingTarget) {
        updateDetectedObjects(detectedObjects);
        updateDrivingTarget(drivingTarget);
    }
}
