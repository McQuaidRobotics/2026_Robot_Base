---
name: frc-loop-overrun
description: Diagnose and fix loop overruns ("Loop time of 0.02s overrun", "CommandScheduler loop overrun", high CPU, laggy/stuttering robot) in FRC WPILib Java robot code — by profiling the real roboRIO over JMX/JFR (the port VisualVM uses), reading riolog overrun epochs, or profiling the desktop simulation, then applying behavior-preserving fixes (CTRE StatusSignal caching, swerve getState reuse, NetworkTables/Limelight flushes, SmartDashboard, AdvantageKit logging volume, per-loop allocation). Use this whenever someone mentions loop overruns, watchdog/Tracer epoch printouts, robot code running slow, VisualVM/JFR on a roboRIO, or wants to optimize/profile FRC robot code performance, even if they don't say "overrun".
---

# FRC loop-overrun profiling & fixing

The robot loop has a 20 ms budget. Overruns mean the main thread used it all, either through its own work or because it was starved (both RIO cores saturated, lock contention with NT/CTRE threads, GC). The job is to **measure where the time actually goes on the hardware, then remove waste without changing what the robot does**.

Guiding facts learned the hard way:

- **Sim is not the robot.** A desktop is roughly 20x faster than a roboRIO, so the sim rarely overruns. Sim also runs fake devices (CTRE sim/HAL sim can be 40%+ of sim samples) and skips every `*Real` IO class. Use the sim for *relative* cost of shared code and for allocation hotspots. Use the RIO for truth.
- **Overrun epochs that jump around randomly** (LED 9 ms one loop, Intake 12 ms the next) mean starvation or contention, not one slow method. Check whole-process CPU before blaming a subsystem.
- **GC pauses stop every thread.** The RIO runs SerialGC on a small heap. A loop already at about 14 ms overruns on any 6–14 ms young-GC pause, so per-loop allocation (log key strings, boxed streams, per-tick lists) is an overrun cause, not just tidiness. `jfr_breakdown.py` prints GC count and pause times.
- **CTRE runs the swerve telemetry callback while holding the drivetrain state lock.** Every `getState()` on the main thread waits on it, so a slow `registerTelemetry` callback shows up as random main-loop stalls even though it runs on another thread.
- **Profiling adds load.** With little CPU headroom, a JFR recording or JMX thread sampling can itself cause overruns. Count "Loop time" lines in the log *outside* recording windows (note the count before and after each recording) before claiming overruns got better or worse.
- **AdvantageKit already batches.** `Logger.recordOutput` only writes to an in-memory table; once per loop `periodicAfterUser` clones it and hands it to a receiver thread (WPILOG/NT4), and NT batches sends itself. There is no "NoFlush then Flush" trick for logs like there is for Limelight. The cost is per key (`writeAllowed`, `put`, `clone`), so only fewer keys helps.
- **Warm up before comparing.** Right after deploy the JIT is cold and everything looks slower. Profile at least 3–4 minutes after code start, in the same robot state (disabled vs. teleop) as the baseline.

## Workflow

### 1. Read what the robot already tells you

If a robot is reachable (`10.TE.AM.2`, or ask for the address), SSH works as `admin` with no password (`ssh -o BatchMode=yes -o StrictHostKeyChecking=no admin@<ip>`). Busybox tools only: no `paste`/`join`/`--time-style`.

If every port is "connection refused" or closed, check the address isn't **this computer's own IP** (`Get-NetIPAddress` / `ipconfig`). A laptop statically set to `10.TE.AM.2` answers for the robot. Have the user change the laptop (e.g. `10.TE.AM.5`, mask `255.0.0.0`).

- `cat /home/lvuser/FRC_UserProgram.log`: WPILib prints per-section epoch timings for every overrun (`robotPeriodic()`, `SmartDashboard.updateValues()`, and per-subsystem `X.periodic()` under "CommandScheduler loop overrun"). Note which sections are big and whether they're consistent or random.
- `cat /home/lvuser/robotCommand`: JVM flags, and whether the JMX flags (`-Dcom.sun.management.jmxremote.port=...`) are present.
- `ssh admin@<ip> 'sh -s' < scripts/rio_thread_cpu.sh`: per-thread user/sys CPU of the JVM plus memory. Two cores means about 200% max. High **sys%** points at syscall-heavy work (CAN, NT, network).
- `ssh admin@<ip> 'sh -s' < scripts/rio_all_threads.sh`: whole-RIO view (every process, IRQs, kernel threads) as % of one core over a timed window, plus overall user/sys/softirq/idle. Use this to answer "what is eating my CPU". Don't trust a single `top -bn1` snapshot for idle %; it swung from 7% to 38% idle on the same robot. Don't hand-roll a fork-per-thread `/proc` loop either: it takes seconds per snapshot and skews the window.
  - Threads inside the JVM named plain `java` are native (CTRE, ntcore, HAL). Map OS tids to Java names with `jfr print --events jdk.ThreadDump rec.jfr` (`nid=0x...` is the tid in hex). A JVM thread in `epoll_wait` that isn't in the dump is the **NetworkTables network thread**, and it can out-use the main loop (22% vs 18.5% of a core on a robot with 4 Limelights plus dashboards). `Thread-2` with no Java stack is CTRE's swerve odometry thread. `can_recv`/`can_stat` plus `irq/53` are CAN; `irq/..-eth0` is network.
  - `lvrt` / `MainAppThread` is the NI LabVIEW runtime, and it is **the parent that launches and restarts the robot program** (`lvrt → frcRunRobot.sh → java`; its PID changes on every deploy). It costs about 2.5% of a core and ~37 MB. Never recommend disabling it.

### 2. Profile the real robot (preferred)

JMX must be enabled in `build.gradle`'s `frcJava` artifact `jvmArgs` (`jmxremote=true`, `.port=1198`, `.authenticate=false`, `.ssl=false`, `java.rmi.server.hostname=<rio ip>`). If it isn't, add it, tell the user, and redeploy. The RIO JRE has no `jcmd`, so drive Flight Recorder remotely with the bundled scripts. Run them with the WPILib JDK (e.g. `C:\Users\Public\wpilib\<year>\jdk\bin\java`):

```
java scripts/RioThreads.java 10.TE.AM.2:1198 10          # per-Java-thread CPU, stack, GC % and heap
java scripts/RioJfr.java     10.TE.AM.2:1198 60 rio.jfr  # 5 ms sampling JFR, downloaded locally
python scripts/jfr_breakdown.py rio.jfr --thread main    # busy %, self & inclusive frames
```

`jfr_breakdown.py` reports **busy %** (share of loop time not idle-waiting). That's the number to drive down and to compare before/after. Use `--filter <package>` to zoom into the team's own code. Use `--thread "robot main"` for sim recordings.

### 3. Sim fallback (no robot available)

Run `./gradlew simulateJava` in the background (set `JAVA_HOME` to the WPILib JDK). Find the PID with `jps -l`. Then `jcmd <pid> JFR.start settings=profile duration=60s filename=...`. Ask the user to put the sim in Teleop and drive or shoot; disabled mode exercises very little. For better resolution copy `lib/jfr/profile.jfc` and set `jdk.ExecutionSample` / `jdk.NativeMethodSample` periods to `1 ms`. Also look at `jdk.ObjectAllocationSample` grouped by first team-code frame. Allocation pressure becomes GC pauses on the RIO's tiny heap.

### 4. Fix — biggest measured bucket first

Read the code behind each hot frame before editing. Every fix must preserve behavior: the same values, the same logs (keys and final values), and the same commands. Common culprits, with the safe fix for each:

| Hot frame / symptom | Cause | Behavior-preserving fix |
|---|---|---|
| `ParentDevice.lookupStatusSignal` / `commonLookup` | Calling `motor.getPosition()`, `getVelocity()` etc. every loop: a synchronized map lookup per call. On a real RIO profile this was ~14% of main-loop busy time, as much as the refresh itself | Do this even without a profile when IO classes call getters in periodic or hot getters: store the `StatusSignal` in a final field at construction (type it `StatusSignal<?>` or the concrete unit type; `BaseStatusSignal` has no `refresh()`) and call `signal.refresh().getValueAsDouble()` where the getter was. Same object, same freshness. For several signals logged together, `BaseStatusSignal.refreshAll(...)` once. A helper like `Log.logMotor(path, motor)` can cache per motor in an `IdentityHashMap`. |
| `StatusSignal.refresh` doubled | `getVelocity().refresh()`: Phoenix 6 getters already refresh by default | Drop the extra `.refresh()`. |
| `SwerveDrivetrain.getState` / `updateFromJni` | `swerve.getState()` called many times per loop; each is a JNI round trip | Read it once at the top of `robotPeriodic` (or the command) and pass the pose/state down. |
| `NetworkTableInstance.flush` / Limelight | `LimelightHelpers.SetRobotOrientation` flushes NT **per camera** | `SetRobotOrientation_NoFlush` for every camera, then a single `LimelightHelpers.Flush()` before reading estimates. |
| `SmartDashboard.putData` in a periodic / telemetry callback | Re-registering a Sendable every call; synchronized, contends with `updateValues()` | Register once (constructor). Mechanism2d/Field2d updates still publish. |
| CTRE swerve `registerTelemetry` callback heavy | Runs on the odometry thread at 100–250 Hz, competing for the 2 cores **and holding the drivetrain state lock** | Precompute anything static (tag poses etc.). Move one-time registration to the constructor. No per-tick allocation. For data the main thread mutates (e.g. a vision tag-id list), fix it at the source: publish an immutable snapshot (`volatile` + `List.copyOf`) rather than reading a live `ArrayList` from another thread. |
| `DriverStation.getStickButton` building warning strings | A binding uses a button index the connected controller doesn't have, or a controller is unplugged: a warning gets built every loop | Flag it to the user (check DS console and bindings). Don't change bindings yourself. |
| `Trigger.<init>` every loop | `RobotModeTriggers.test().getAsBoolean()` allocates a Trigger per call | `DriverStation.isTestEnabled()` (same check). |
| Boxed streams in hot paths | `Collections.max(Arrays.stream(arr).boxed().toList())` + `indexOf` | Plain index loop returning the *first* max/min, which is what `indexOf` gave. Add a tiny unit test. |
| `LogTable.writeAllowed` / `put` / `clone`, `Logger.periodicAfterUser` | AdvantageKit cost scales with number of keys logged per loop | Remove only **exact duplicates** (same key written twice in one loop; AK keeps the last value). Cutting real log keys or `NT4Publisher` is a team decision: recommend it with numbers, don't do it unasked. |
| `DriverStation.reportError/Warning` in periodic | Synchronous DS message each loop | Only report on state change. |
| Blocking CTRE config calls (`setPosition`, `getConfigurator().apply`) | Wait for a CAN ack (up to ~100 ms) | Fine one-shot. Flag it if any command calls it every loop. |

Don't "fix" things the profile doesn't show. Micro-optimizing a 0.1% frame adds diff without moving overruns.

### 5. Verify

- `./gradlew build` (runs tests and spotless). All tests must still pass. If a test fails, rerun it on the untouched code (`git stash -u`, test, `git stash pop`) to see whether it was already failing. A pre-existing failure isn't yours to fix silently: tell the user, and deploy with `./gradlew deploy -x test` only after confirming your own tests pass.
- Don't touch unrelated working-tree changes (e.g. a team number edited in `.wpilib/wpilib_preferences.json`); mention them.
- If a robot is connected **and the user has asked you to work on it**, deploy with `./gradlew deploy` (add `-ProborioAddress=<ip>` if the build supports it). Wait for "Robot program startup complete" in the log, warm up 3–4 min, re-profile with the same settings and duration as the baseline, and compare busy % plus the specific frames you targeted. A deploy restarts robot code (the robot comes back disabled), so make sure that's expected.
- Network blips happen: a timed-out SSH mid-measurement invalidates any before/after counter it was part of. Re-check before reporting numbers.

## Report back

Lead with the measured result: busy % of the 20 ms loop before vs. after, normalized to the same sample window, and whether overruns still occur. Then give a short table of the main buckets. List each fix in one line with why it's behavior-preserving. Finish with the remaining biggest levers that need a team decision (usually logging volume, SmartDashboard widgets, NT4 publishing during matches, number of NT clients like Limelights and dashboards, and CTRE status signal rates via `setUpdateFrequency`/`optimizeBusUtilization`). Be honest when the profile can't prove something, e.g. sim-only measurements can't confirm RIO overruns are gone. Don't commit unless asked.
