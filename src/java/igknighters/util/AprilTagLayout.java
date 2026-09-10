package igknighters.util;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class AprilTagLayout {

    /**
     * Deploy relative path to the AprilTag field layout.
     *
     * <p>The field layout changes every season. Replace this file with the current game's layout
     * (the WPILib or AndyMark published JSON) when starting a new season.
     */
    public static final String LAYOUT_PATH = "assets/apriltag_layout.json";

    private final Map<Integer, Pose3d> tagPoses = new HashMap<>();

    public AprilTagLayout() throws IOException {
        File file = new File(Filesystem.getDeployDirectory(), LAYOUT_PATH);
        ObjectMapper mapper = new ObjectMapper();
        mapper.configure(
                com.fasterxml.jackson.databind.DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES,
                false);
        JsonLayout layout = mapper.readValue(file, JsonLayout.class);

        for (JsonTag tag : layout.tags) {
            tagPoses.put(
                    tag.id,
                    new Pose3d(
                            new Translation3d(
                                    tag.pose.translation.x,
                                    tag.pose.translation.y,
                                    tag.pose.translation.z),
                            new Rotation3d(
                                    new Quaternion(
                                            tag.pose.rotation.quaternion.w,
                                            tag.pose.rotation.quaternion.x,
                                            tag.pose.rotation.quaternion.y,
                                            tag.pose.rotation.quaternion.z))));
        }
    }

    public Map<Integer, Pose3d> getTagPoses() {
        return tagPoses;
    }

    // Inner classes for JSON deserialization
    private static class JsonLayout {
        @JsonProperty("tags")
        public List<JsonTag> tags;

        @JsonProperty("field")
        public JsonField field;
    }

    private static class JsonField {
        @JsonProperty("length")
        public double length;

        @JsonProperty("width")
        public double width;
    }

    private static class JsonTag {
        @JsonProperty("ID")
        public int id;

        @JsonProperty("pose")
        public JsonPose pose;
    }

    private static class JsonPose {
        @JsonProperty("translation")
        public JsonTranslation translation;

        @JsonProperty("rotation")
        public JsonRotation rotation;
    }

    private static class JsonTranslation {
        @JsonProperty("x")
        public double x;

        @JsonProperty("y")
        public double y;

        @JsonProperty("z")
        public double z;
    }

    private static class JsonRotation {
        @JsonProperty("quaternion")
        public JsonQuaternion quaternion;
    }

    private static class JsonQuaternion {
        @JsonProperty("W")
        public double w;

        @JsonProperty("X")
        public double x;

        @JsonProperty("Y")
        public double y;

        @JsonProperty("Z")
        public double z;
    }
}
