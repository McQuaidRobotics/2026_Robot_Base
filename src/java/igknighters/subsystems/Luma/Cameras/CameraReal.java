package igknighters.subsystems.Luma.Cameras;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import igknighters.Robot;
import igknighters.util.log.Log;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CameraReal extends Camera {

    /**
     * Height above the floor of the center of the tracked object, in meters.
     *
     * <p>For a spherical game piece this is its radius. Set this to match the object the detection
     * pipeline is trained on each season.
     */
    public static final double TARGET_HEIGHT_METERS = 0.075;

    PhotonCamera camera;
    String name;
    double cameraHeightMeters;
    double cameraPitchRadians = 0.0;
    Translation2d robotToCameraTranslation;

    List<PhotonPipelineResult> results = new ArrayList<>();
    List<Translation2d> gamePieceTranslations = new ArrayList<>();
    boolean noObjects = true; // Default to true

    public CameraReal(
            String cameraName, double cameraHeightMeters, Translation2d robotToCameraTranslation) {
        this.camera = new PhotonCamera(cameraName);
        this.name = cameraName;
        this.cameraHeightMeters = cameraHeightMeters;
        this.robotToCameraTranslation = robotToCameraTranslation;

        camera.setPipelineIndex(0);

        if (!Robot.consts.limelightVision().disableVisionLogs()) {
            Log.log(cameraName, true);
            Log.log("ROBOT/Subsystems/Vision/" + cameraName + "/Status", "ENABLED");
        }
    }

    public CameraReal(String cameraName) {
        // placeholder values, the camera itself is 5cm tall
        this(cameraName, 0.05, new Translation2d());
    }

    @Override
    public void periodic() {
        if (!Robot.consts.limelightVision().disableVisionLogs()) {
            Log.log("ROBOT/Subsystems/Vision/" + name + "/Connected", camera.isConnected());
        }

        // Removed unnecessary new ArrayList<>() allocation
        List<PhotonPipelineResult> potentialResults = camera.getAllUnreadResults();

        // Simplified sticky-fault/memory logic
        if (!potentialResults.isEmpty()) {
            results = potentialResults;
            noObjects = false;
        } else {
            // If it was already empty, keep it empty. If it wasn't, now it is.
            if (noObjects) {
                results = potentialResults; // Clear the cached results
            }
            noObjects = true;
        }

        // Update translations
        getTargetTranslations();
    }

    @Override
    public void simulationPeriodic() {}

    public Translation2d getGamePieceOffsetFromTargetList(List<PhotonTrackedTarget> targets) {
        if (targets.isEmpty()) {
            throw new IllegalArgumentException(
                    "Target list is empty in getGamePieceOffsetFromTargetList");
        }

        // Sort mutates the list, but we already made a safe copy in getGamePieceOffset()
        targets.sort(Comparator.comparingDouble(PhotonTrackedTarget::getArea));

        var bestTarget = targets.get(targets.size() - 1);
        double distance =
                PhotonUtils.calculateDistanceToTargetMeters(
                        cameraHeightMeters,
                        TARGET_HEIGHT_METERS,
                        cameraPitchRadians,
                        Units.degreesToRadians(bestTarget.getPitch()));

        double yaw = Units.degreesToRadians(bestTarget.getYaw());
        return new Translation2d(distance * Math.cos(yaw), distance * Math.sin(yaw))
                .plus(robotToCameraTranslation);
    }

    @Override
    public Translation2d getGamePieceOffset() {
        if (!Robot.consts.limelightVision().disableVisionLogs()) {
            Log.log("ROBOT/Subsystems/Vision/Getting Offset", true);
        }

        if (results.isEmpty()) {
            if (!Robot.consts.limelightVision().disableVisionLogs()) {
                Log.log("ROBOT/Subsystems/Vision/ObjectDetection/Camera Results", false);
            }
            return new Translation2d();
        }

        if (!Robot.consts.limelightVision().disableVisionLogs()) {
            Log.log("ROBOT/Subsystems/Vision/ObjectDetection/Camera Results", true);
        }
        var result = results.get(results.size() - 1);

        if (!result.hasTargets()) {
            if (!Robot.consts.limelightVision().disableVisionLogs()) {
                Log.log("ROBOT/Subsystems/Vision/ObjectDetection/Camera Has Target", false);
            }
            return new Translation2d();
        }

        if (!Robot.consts.limelightVision().disableVisionLogs()) {
            Log.log("ROBOT/Subsystems/Vision/ObjectDetection/Camera Has Target", true);
        }

        // Make a COPY of the targets list before sorting to avoid mutating PhotonVision's internal
        // data
        List<PhotonTrackedTarget> targets = new ArrayList<>(result.getTargets());
        targets.sort(Comparator.comparingDouble(PhotonTrackedTarget::getYaw));

        List<PhotonTrackedTarget> bestCluster = new ArrayList<>();
        List<PhotonTrackedTarget> currentCluster = new ArrayList<>();

        double clusterThresholdDeg = 10.0;

        for (var t : targets) {
            if (currentCluster.isEmpty()) {
                currentCluster.add(t);
                continue;
            }

            var last = currentCluster.get(currentCluster.size() - 1);

            if (Math.abs(t.getYaw() - last.getYaw()) <= clusterThresholdDeg) {
                currentCluster.add(t);
            } else {
                if (currentCluster.size() > bestCluster.size()) {
                    bestCluster = new ArrayList<>(currentCluster);
                }
                currentCluster.clear();
                currentCluster.add(t);
            }
        }

        if (currentCluster.size() > bestCluster.size()) {
            bestCluster = currentCluster;
        }

        if (!Robot.consts.limelightVision().disableVisionLogs()) {
            Log.log(
                    "ROBOT/Subsystems/Vision/ObjectDetection/Camera Cluster Size",
                    bestCluster.size());
        }

        return getGamePieceOffsetFromTargetList(bestCluster);
    }

    @Override
    public Pose2d getRobotPose() {
        return new Pose2d();
    }

    @Override
    public String getName() {
        return "CameraReal-" + camera.getName();
    }

    @Override
    public List<Translation2d> getTargetTranslations() {
        // MEMORY LEAK FIXED: Clear the list at the start of the loop
        gamePieceTranslations.clear();

        for (int resultNumber = 0; resultNumber < results.size(); resultNumber++) {
            PhotonPipelineResult gamePieces = results.get(resultNumber);

            for (int gamePieceNumber = 0;
                    gamePieceNumber < gamePieces.getTargets().size();
                    gamePieceNumber++) {
                PhotonTrackedTarget gamePiece = gamePieces.getTargets().get(gamePieceNumber);

                if (!Robot.consts.limelightVision().disableVisionLogs()) {
                    Log.log(
                            "Subsystems/Vision/ObjectDetection/GAMEPIECES/"
                                    + gamePieceNumber
                                    + "/pitch",
                            gamePiece.pitch);
                    Log.log(
                            "Subsystems/Vision/ObjectDetection/GAMEPIECES/"
                                    + gamePieceNumber
                                    + "/yaw",
                            gamePiece.yaw);
                }

                double distance =
                        PhotonUtils.calculateDistanceToTargetMeters(
                                cameraHeightMeters,
                                TARGET_HEIGHT_METERS,
                                cameraPitchRadians,
                                Units.degreesToRadians(gamePiece.getPitch()));

                double yaw = Units.degreesToRadians(gamePiece.getYaw());

                Translation2d gamePieceTranslation =
                        new Translation2d(distance * Math.cos(yaw), distance * Math.sin(yaw))
                                .plus(robotToCameraTranslation);

                if (!Robot.consts.limelightVision().disableVisionLogs()) {
                    Log.log(
                            "Subsystems/Vision/ObjectDetection/GAMEPIECES/"
                                    + gamePieceNumber
                                    + "/translation",
                            gamePieceTranslation);
                }

                gamePieceTranslations.add(gamePieceTranslation);
            }
        }
        return gamePieceTranslations;
    }
}
