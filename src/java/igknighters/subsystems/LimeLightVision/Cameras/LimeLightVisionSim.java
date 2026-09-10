package igknighters.subsystems.LimeLightVision.Cameras;

import edu.wpi.first.math.geometry.Pose2d;
import igknighters.subsystems.LimeLightVision.Helpers.VisionSimulator;
import igknighters.util.Merging.PoseAverager;
import java.util.ArrayList;
import java.util.List;

public class LimeLightVisionSim extends LimeLights {
    List<VisionSimulator> visionSimulators;
    List<Pose2d> estimatedPoses;
    List<Double[]> cameraRotations;
    List<Integer> visibleTagIds = new ArrayList<>();
    double timesum = 0.0;
    double camerasThatSeeStuff = 0.0;

    double cameraFOV = 60;

    public LimeLightVisionSim(String... cameraNames) {
        visionSimulators = new ArrayList<>();
        estimatedPoses = new ArrayList<>();
        cameraRotations = new ArrayList<>();
        cameraRotations.add(new Double[] {60.0, 120.0}); // front cam
        cameraRotations.add(new Double[] {135.0, 195.0});
        cameraRotations.add(new Double[] {150.0, 210.0});
        cameraRotations.add(new Double[] {240.0, 300.0});
        // these are how far they should be off from irl
        for (int i = 0; i < cameraRotations.size(); i++) {
            // Initialize each camera
            visionSimulators.add(
                    new VisionSimulator(
                            cameraRotations.get(i)[0],
                            cameraRotations.get(i)[1],
                            5,
                            0.0001,
                            0.0,
                            0.1));
        }
    }

    @Override
    public Pose2d getRobotPoseFromVision(
            double yaw,
            double yawRate,
            double pitch,
            double pitchRate,
            double roll,
            double rollRate) {
        estimatedPoses.clear();
        visibleTagIds.clear();
        timesum = 0.0;
        camerasThatSeeStuff = 0.0;
        for (VisionSimulator simulator : visionSimulators) {
            Pose2d estimatedPose = simulator.getEstimatedPose();
            if (estimatedPose != null) {
                estimatedPoses.add(estimatedPose);
                timesum += simulator.getTime();
                camerasThatSeeStuff++;
                visibleTagIds.addAll(simulator.getVisibleTagIds());
            }
        }
        return PoseAverager.averagePose2ds(estimatedPoses);
    }

    @Override
    public void enableCameras(int imu_mode) {}

    @Override
    public void saveCameras() {}

    @Override
    public double getLastTimeStamp() {
        // In simulation, we can get the timestamp from the latest results
        return timesum / camerasThatSeeStuff;
    }

    @Override
    public double timeSinceLastSample() {
        return 0.0;
    }

    @Override
    public List<Integer> getVisibleTagIds() {
        return visibleTagIds;
    }
}
