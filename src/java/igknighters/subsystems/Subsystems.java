package igknighters.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.subsystems.LimeLightVision.Localization.LocalizationVision;
import igknighters.subsystems.LimeLightVision.ObjectDetection.ObjectDetection;
import igknighters.subsystems.led.Led;
import igknighters.subsystems.swerve.Swerve;

/**
 * Container for the robot's subsystems.
 *
 * <p>The robot base holds only the season independent subsystems. Add game specific superstructure
 * mechanisms here in a season repository.
 */
public class Subsystems {
    public final Swerve swerve;
    public final LocalizationVision vision;
    public final ObjectDetection objectDetection;
    public final Led led;
    public final SubsystemBase[] lockedResources;

    public Subsystems(
            Swerve swerve, LocalizationVision vision, ObjectDetection objectDetection, Led led) {
        this.swerve = swerve;
        this.vision = vision;
        this.objectDetection = objectDetection;
        this.led = led;
        this.lockedResources = new SubsystemBase[] {swerve, vision, objectDetection, led};
    }
}
