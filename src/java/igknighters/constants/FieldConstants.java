package igknighters.constants;

import igknighters.commands.Repulsor.obstacle;
import java.util.ArrayList;

/**
 * Field geometry shared by the navigation stack.
 *
 * <p>The robot base intentionally carries no game element poses (scoring locations, climb targets,
 * field features). Add those in a season repository alongside the mechanisms that use them.
 */
public class FieldConstants {

    /** Field length along the X axis, in meters. */
    public static final double X_FIELD = 651.12 * Conv.INCHES_TO_METERS;

    /** Field width along the Y axis, in meters. */
    public static final double Y_FIELD = 316.64 * Conv.INCHES_TO_METERS;

    /**
     * Repulsor obstacles used by the navigation stack.
     *
     * <p>Empty in the robot base. Populate {@link OBSTACLES#ALL_OBSTACLES} with a season's field
     * features to make the repulsor planner avoid them.
     */
    public static class OBSTACLES {
        public static final ArrayList<obstacle> ALL_OBSTACLES = new ArrayList<>();
    }
}
