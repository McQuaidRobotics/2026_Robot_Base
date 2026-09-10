package vroom.Fields;

import java.util.ArrayList;
import vroom.Field;
import vroom.Obstacles.Obstacle;

/**
 * A field with no obstacles.
 *
 * <p>This is the robot base default so the repulsor navigation stack is wired up and runnable
 * without carrying any game specific geometry. Add a season's obstacles by implementing {@link
 * Field} alongside this class and swapping it in where this is constructed.
 */
public class EmptyField implements Field {
    private final ArrayList<Obstacle> obstacleObjects = new ArrayList<>();

    @Override
    public ArrayList<Obstacle> getObstacles() {
        return obstacleObjects;
    }
}
