# Wayfinder Library Technical Manual

The Wayfinder library is a real-time path planning and kinematic constraint engine designed for FRC swerve drivetrains. It combines **Potential Field Navigation** with a **High-Fidelity Setpoint Generator** to provide smooth, collision-free movement that respects the physical limits of the robot.

## Core Architecture

### 1. Repulsor Field Planner (`wayfinder.repulsorField`)

Unlike traditional path planners (like Choreo or PathPlanner) that pre-calculate a static path, Wayfinder calculates the robot's heading and velocity in real-time based on "forces."
*   **Attraction:** The target pose exerts an attractive force, pulling the robot toward the goal.
*   **Repulsion:** Obstacles exert repulsive forces that push the robot away.
*   **Resultant Vector:** The planner sums these forces to determine the "Intermediate Pose"—the immediate direction the robot should move to reach the goal while avoiding obstacles.

### 2. Swerve Setpoint Generator (`wayfinder.setpointGenerator`)
Once the desired velocity is determined, the Setpoint Generator ensures the robot can actually achieve it. It monitors:
*   **Motor Torque & Current:** Prevents brownouts and motor overheating.
*   **Traction (Friction):** Ensures module forces do not exceed the Coefficient of Friction (CoF), preventing wheel slip.
*   **Kinematic Limits:** Respects maximum module rotation speeds and chassis acceleration.
*   **Advanced States:** Outputs `AdvancedSwerveModuleState`, which includes required accelerations for more precise torque control.

---

## Integration with CTRE Swerve (Phoenix 6)

To integrate Wayfinder with a CTRE-based `CommandSwerveDrivetrain`, follow these steps:

### 1. Initialization
In your `Swerve` subsystem, initialize the generator with your robot's physical constants:

```java
SwerveSetpointGenerator generator = new SwerveSetpointGenerator(
    moduleLocations,
    driveMotor,        // DCMotorExt for torque curves
    angleMotor,
    statorCurrentLimit,
    supplyCurrentLimit,
    massKg,
    moiKgMetersSquared,
    wheelDiameterMeters,
    wheelCoF,          // e.g., 1.1 for TPU/Colson on carpet
    torqueLoss         // Internal drivetrain friction
);
```

### 2. The Control Loop
In your command's `execute()` method:
1.  **Calculate Forces:** Get the desired `FieldSpeeds` from `RepulsorFieldPlanner.calculate()`.
2.  **Constraint Speeds:** Pass those speeds into `generator.generateSetpoint()`.
3.  **Apply to Hardware:** Use the `SwerveSetpoint` to update your `SwerveRequest`.

```java
// Get the safe speeds from the generator
SwerveSetpoint setpoint = generator.generateSetpoint(
    prevSetpoint,
    currentHeading,
    desiredSpeeds,
    constraints,
    dt
);

// Convert to CTRE Request
drivetrain.setControl(
    m_applySpeeds.withSpeeds(setpoint.speeds().toWpilib())
);
```

---

## Field Maintenance & Obstacles

Wayfinder uses geometric primitives to represent field elements. These are defined in `wayfinder.repulsorField.Obstacle`.

### Obstacle Catalog
| Type | Use Case |
| :--- | :--- |
| `VerticalObstacle` / `HorizontalObstacle` | Field walls, long straight barriers (e.g., the Source or Wing walls). |
| `TeardropObstacle` | Complex elements where the robot should "flow" around. It has a radius and a "tail" to guide the robot into a specific orientation or path. |
| `SnowmanObstacle` | Compound circular obstacles for multi-part field elements (e.g., the 2024 Stage). |

### Updating for Future Years
When a new game is released:
1.  **Map the Field:** Identify the coordinates (meters) of major obstacles.
2.  **Define Constants:** Update your `FieldConstants` class with a new list of `Obstacle` objects.
3.  **Assign Strengths:**
    *   `strength`: How hard the obstacle pushes. Start at `1.0`.
    *   `maxRange`: How far away the robot starts to feel the push. Usually `0.5` to `1.5` meters.
4.  **Visualize:** Use the `RepulsorFieldPlanner.getArrows()` method to visualize the field in AdvantageScope to ensure there are no "dead zones" where the robot gets stuck.

## WayfinderManager: The Easy Interface

For easy maintenance, use the `WayfinderManager` class. it consolidates all library features into a single, static-accessible API.

### Usage Example:
```java
// 1. Setup (in RobotContainer or Subsystem constructor)
WayfinderManager.setup(myPositionalController, mySetpointGenerator);

// 2. Add Obstacles
WayfinderManager.addObstacle(new Obstacle.TeardropObstacle(...));

// 3. Toggle Debugging
WayfinderManager.setArrowsEnabled(true);

// 4. Calculate (in your command's execute)
SwerveSetpoint setpoint = WayfinderManager.calculate(dt, pose, speeds, target, constraints);
```

---

## Performance Tuning

*   **Wheel Slip:** If the robot is drifting during high-accel turns, lower the `wheelCoF` in the generator.
*   **Oscillation:** If the robot wobbles near obstacles, reduce the `strength` of the obstacles or increase the `PositionalController` D-gain.
*   **Latency:** Ensure Wayfinder is running at the same frequency as your odometry (e.g., 50Hz or 100Hz).
