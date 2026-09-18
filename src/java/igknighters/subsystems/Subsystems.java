package igknighters.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import igknighters.subsystems.LimeLightVision.LimeLightVision_OLD;
import igknighters.subsystems.Luma.Luma;
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
    public final LimeLightVision_OLD vision;
    public final Led led;
    public final Luma luma;
    public final SubsystemBase[] lockedResources;

    public Subsystems(Swerve swerve, LimeLightVision_OLD vision, Led led, Luma luma) {
        this.swerve = swerve;
        this.vision = vision;
        this.led = led;
        this.luma = luma;
        this.lockedResources = new SubsystemBase[] {swerve, luma, vision, led};
    }
}
