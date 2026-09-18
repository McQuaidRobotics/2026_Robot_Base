package igknighters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import igknighters.util.TunableValues;
import igknighters.util.TunableValues.TunableBoolean;
import java.util.List;

/**
 * FieldVisualizer provides methods to update specific objects on the field (Field2d) independently
 * of the main telemetry loop. This allows for updating vision targets, detected objects, and other
 * field elements only when necessary.
 *
 * <p>The robot base publishes only season independent overlays. Add publishers for game specific
 * targets and mechanism poses in a season repository.
 */
public class FieldVisualizer {

    private FieldVisualizer() {
        fieldTypePub.set("Field2d");
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

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Pose");
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    private final DoubleArrayPublisher predictedFuturePose =
            table.getDoubleArrayTopic("futurePose").publish();

    private final DoubleArrayPublisher drivingTargetPub =
            table.getDoubleArrayTopic("drivingTargetPose").publish();

    private final DoubleArrayPublisher detectedObjectsPub =
            table.getDoubleArrayTopic("detectedObjects").publish();

    public void updatePredictedPose(Pose2d pred_pose) {
        if (pred_pose == null) {
            predictedFuturePose.set(new double[0]);
            return;
        }
        predictedFuturePose.set(
                new double[] {
                    pred_pose.getX(), pred_pose.getY(), pred_pose.getRotation().getDegrees()
                });
    }

    /**
     * Updates the driving target pose on the field.
     *
     * @param target The pose of the driving target, or null to clear.
     */
    public void updateDrivingTarget(Pose2d target) {
        if (target == null || !shouldShowDrivingTarget.value()) {
            drivingTargetPub.set(new double[0]);
            return;
        }
        drivingTargetPub.set(
                new double[] {target.getX(), target.getY(), target.getRotation().getDegrees()});
    }

    /**
     * Updates the list of detected objects on the field.
     *
     * @param objects A list of poses for detected objects, or null/empty to clear.
     */
    public void updateDetectedObjects(List<Pose2d> objects) {
        if (objects == null || objects.isEmpty() || !shouldShowDetectedObjects.value()) {
            detectedObjectsPub.set(new double[0]);
            return;
        }
        int i = 0;
        double[] array = new double[objects.size() * 3];
        for (Pose2d obj : objects) {
            array[i++] = obj.getX();
            array[i++] = obj.getY();
            array[i++] = obj.getRotation().getDegrees();
        }
        detectedObjectsPub.set(array);
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
