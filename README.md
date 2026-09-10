# 2026_Robot_Base

Season independent robot base for McQuaid Robotics, extracted from the 2026 competition
code (`2026_Rebuilt`).

## What lives here

Everything that is not tied to a specific game:

- **Drivetrain** — CTRE swerve (`subsystems/swerve`), teleop drive commands, setpoint generation
- **Vision** — Limelight AprilTag pose estimation (`subsystems/LimeLightVision`) and Luma object
  detection (`subsystems/Luma`)
- **LEDs** — `subsystems/led`
- **Navigation** — `wayfinder` (controllers, pose estimation, setpoint generator) and `vroom`
  (repulsor path planning)
- **Simulation** — `sham` physics simulation
- **Logging** — `monologue`, AdvantageKit wiring, `Telemetry`, `FieldVisualizer`
- **Utilities** — `wpilibExt`, tunable values, pose prediction, channels

## What does not live here

Game specific superstructure mechanisms, their commands, and their paths. That means no
shooter/intake/indexer subsystems, no scoring commands, no auto routines, and no Choreo
trajectories. Those belong in the season repository that builds on this base.

## Starting a season

1. Fork or branch this repository for the new season.
2. Replace `src/main/deploy/assets/apriltag_layout.json` with the current game's AprilTag layout.
3. Add superstructure subsystems and register them in `subsystems/Subsystems.java`.
4. Add game specific field geometry to `constants/FieldConstants.java` and repulsor obstacles to
   `FieldConstants.OBSTACLES.ALL_OBSTACLES`.
5. Add auto routines against the factory from `Robot.getAutoFactory()` and register them on
   `autoChooser`.
6. Add driver bindings in `controllers/DriverController.bind(Subsystems)`.

## Build

```
./gradlew build
```
