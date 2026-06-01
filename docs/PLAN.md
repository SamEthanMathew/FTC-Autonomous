# Pedro Pathing Autonomous Improvements — Plan

## Context

We run an FTC team on **Pedro Pathing** for autonomous. Pedro's path follower
is solid; the tooling *around* it (battery-independent characterization, an
offline test harness, automatic tuning, logging/replay, and vision-fused
localization) is weak. This project adds that tooling as **clean, non-forking
additions layered on top of the real Pedro Pathing library**, built and proven
in dependency order, gated on tests.

The guiding constraint from the brief: **never claim hardware behavior is
verified when it was only compiled or simulated.** Everything that can be
proven on the JVM is unit-tested and green; everything that touches motors,
sensors, or an OpMode lifecycle is written, clearly flagged
`// HARDWARE-VALIDATION REQUIRED`, and listed in a per-phase robot checklist.

## Ground truth (pulled from the real sources, not memory)

Verified by cloning `github.com/Pedro-Pathing/PedroPathing` and `…/Quickstart`
into `/tmp` and reading source, plus Maven Central metadata:

| Fact | Value |
|---|---|
| Pedro Pathing version | **2.1.2** (brief said 1.1.0 — outdated) |
| Distribution | Maven Central: `com.pedropathing:core:2.1.2`, `com.pedropathing:ftc:2.1.2` |
| Module split | `core` = **pure JVM** (Follower, Drivetrain, Localizer, FollowerConstants, Pose, PathBuilder). `ftc` = Android (Mecanum, Swerve, localizers, PoseConverter). |
| FTC SDK | 11.1.0 (Java 8, Gradle 8.9, AGP 8.7.0) |
| `Drivetrain` | **abstract class** `com.pedropathing.drivetrain.Drivetrain`; follower drives via `runDrive(Vector corrective, Vector heading, Vector pathing, double robotHeading, Vector velocity)` → `calculateDrive(...)` → `runDrive(double[])`. |
| `Localizer` | **interface** `com.pedropathing.localization.Localizer` (13 methods). |
| `Follower` ctor | `Follower(FollowerConstants, Localizer, Drivetrain, PathConstraints)` and 3-arg variant; per-loop `update()`; `followPath(Path/PathChain[, holdEnd])`; `isBusy()`; `getPose()`. |
| Decel / mass constants | In **core** `FollowerConstants`: `forwardZeroPowerAcceleration` (−41.278), `lateralZeroPowerAcceleration` (−59.78), `mass` (10.65). |
| Velocity constants | In **ftc** `MecanumConstants`: `xVelocity` (81.34), `yVelocity` (65.43). Exposed in core via `Drivetrain.xVelocity()/yVelocity()`. |
| PIDF holders (core `FollowerConstants`) | `coefficientsTranslationalPIDF`, `coefficientsHeadingPIDF`, `coefficientsDrivePIDF` (Filtered), secondary variants, `centripetalScaling`. |
| **Voltage compensation** | **Already in 2.1.2.** `Drivetrain.useVoltageCompensation/nominalVoltage`; formula in `Mecanum`/`Swerve`: `vNorm = (nomV − nomV·sfc)/(V − (nomV²/V)·sfc)`, `sfc=0 ⇒ nomV/V`. Default OFF. |
| Tuning OpModes | Live in the **Quickstart** (`TeamCode/.../pedroPathing/Tuning.java`): ForwardVelocityTuner, LateralVelocityTuner, Forward/LateralZeroPowerAccelerationTuner, PredictiveBrakingTuner, Translational/Heading/Drive/CentripetalTuner. |

**De-risk done:** a standalone JVM Gradle module depending on
`com.pedropathing:core:2.1.2` compiles and loads the unmodified `Follower`,
`Drivetrain`, and `Localizer` (`pedro-extensions` smoke test green). The
headless follower-in-the-loop strategy is therefore viable.

## §0 assumptions (brief left these blank; chosen as documented, configurable)

- **TEAMCODE_REPO** = this repo, scaffolded fresh (it was empty).
- **FTC_SDK_VERSION** = 11.1.0 (detected).
- **DRIVETRAIN** = **mecanum** (default; swerve extension points called out). The
  plant and follower-in-the-loop are largely drivetrain-agnostic.
- **LOCALIZER_HW** = configurable; OpMode examples default to **goBILDA Pinpoint**.
  All JVM logic is localizer-agnostic (works against the `Localizer` interface).
- **Season (Phase E)** = **DECODE (2025–26)**; exclude Obelisk tags (21–23),
  localize off valid goal tags; the tag→pose map is a **config input** whose
  exact coordinates need confirmation before hardware use.

These are recorded in `PROGRESS.md`; trivial to change if any differ.

## Architecture

Two cooperating trees, separated by what can be proven here:

1. **`pedro-extensions/`** — standalone, pure-JVM Gradle build (`java-library`),
   depends on `com.pedropathing:core` + `commons-math3` + JUnit5. Holds **all
   platform-independent logic and every JUnit test**. `./gradlew test` is green
   here with only a JDK. Packages under `org.firstinspires.ftc.pedroext`:
   `voltage/`, `sim/`, `tuning/`, `logging/`, `fusion/`.
2. **`TeamCode/`** — Android OpModes + `Constants` + thin hardware glue.
   Depends on `com.pedropathing:ftc` + FTC SDK + `pedro-extensions`. **No Android
   SDK in this environment, so these are written + structured to compile on a
   real machine, never claimed as compiled here.** Consumed by an FTC SDK
   project via composite build / `mavenLocal`; documented in TeamCode/README.

Architectural rules honored: extend Pedro's seams (`Drivetrain`, `Localizer`)
— never fork; all new behavior behind config flags with safe (off/pass-through)
defaults; dynamics stored **voltage-normalized**; the optimizer is decoupled via
one `TrajectoryEvaluator` interface implemented twice (sim + on-robot).

## Phases (build A → B → C, gate on tests; then D, E)

### Phase A — Voltage compensation (foundational)
Pedro 2.1.2 **already** normalizes by bus voltage, so we **expose + test +
document**, not reimplement (per §6-A).
- `voltage/VoltageCompensationModel.java`: pure-Java port of Pedro's exact
  formula + clamp to [-1,1], with a documented `sfc=0 ⇒ nomV/V` mode matching
  the brief's simple ratio.
- **JVM tests (green):** known (commanded, V) pairs → expected clamped output;
  edges (V ≥ nominal ⇒ ≤ pass-through; brownout ⇒ clamp); `sfc=0` equivalence.
- **Compile-elsewhere:** `Constants.java` enabling `useVoltageCompensation(true)`
  + `nominalVoltage`.
- **Robot checklist:** straight-line distance repeatability, full vs drained pack.

### Phase B — Dynamics-accurate offline simulator (the test harness)
- `sim/RobotPlant.java`: discrete-time first-order plant. Per-axis steady-state
  velocity = command × (`xVelocity`,`yVelocity`); relaxation time constants
  `τ = vMax / |zeroPowerAccel|` per axis (ties decel to characterized
  constants); heading dynamics; `mass` used for added-mass disturbance. Pure JVM.
- `sim/SimulatedDrivetrain.java` (extends core `Drivetrain`) overrides the
  vector `runDrive(...)` to turn corrective+pathing+heading powers into a plant
  command. `sim/SimulatedLocalizer.java` (implements `Localizer`) mirrors plant
  pose/velocity (live `getPose()`, so loop ordering is robust).
- `sim/SimulationLoop.java`: each tick `follower.update()` then `plant.step(dt)`.
  `sim/Disturbance.java`: position kick / added mass / sensor noise.
  `sim/TraceLogger.java`: per-loop pose, target, correction vectors.
- **JVM tests (green):** real follower on straight / strafe / curved Bézier /
  multi-segment paths converges within tolerance, stays inside an error
  envelope, and recovers from an injected disturbance.
- **Docs:** model assumptions + where it diverges from reality.

### Phase C — Self-calibration + auto-PID
- **C2 first (JVM, the headline test):** `tuning/TrajectoryEvaluator` (gains →
  scalar cost = tracking RMSE + overshoot penalty); `SimTrajectoryEvaluator`
  (uses Phase B); `Optimizer` interface with **Twiddle/coordinate-descent**
  (pure Java) and **CMA-ES** (`commons-math3 CMAESOptimizer`). Respect Pedro's
  exact coefficient structure (translational/heading/drive PIDF + centripetal).
  - **JVM tests (green):** from a deliberately bad gain set, both optimizers cut
    RMSE below threshold; stable across seeds.
- **C1 (compile-elsewhere):** `SelfCalibrationOpMode` running the velocity +
  zero-power tuners back-to-back, N runs (default 5), outlier rejection, average,
  **write results into Constants** (patch a generated values file); guard rails
  (min distance; warn if velocity still climbing). `// HARDWARE-VALIDATION REQUIRED`.
- **C2 on-robot (compile-elsewhere):** on-robot `TrajectoryEvaluator` + auto-tune
  OpMode logging each trial's gains+cost (resumable). Same optimizer code.

### Phase D — File logging + replay (stretch)
- `logging/` pure-Java serializer (per-loop pose, target, 4 correction vectors,
  commanded vs applied power, battery V) → AdvantageScope-readable schema (or a
  documented custom schema + converter). **JVM tests** for the serializer.
- Android file-write OpMode glue: `// HARDWARE-VALIDATION REQUIRED`.

### Phase E — Vision-fused localization (stretch, highest value)
- `fusion/PoseEKF.java` (commons-math3 linear algebra): covariance-weighted
  AprilTag updates, trust ∝ tag proximity/centering, fix-quality gate, smooth
  feedback (no pose discontinuity). `fusion/FusingLocalizer` wraps an odometry
  `Localizer`. Tag map is a **config input**; DECODE-valid tags only.
- **JVM tests (green):** synthetic odometry+vision vs known ground truth — bounded
  drift, no jumps, recovery from injected collision-style jumps.
- Android VisionPortal/AprilTag glue: `// HARDWARE-VALIDATION REQUIRED`.

## Verification

- Each phase: `cd pedro-extensions && ./gradlew test` green for that phase's JVM
  tests (CI-style, JDK-only, no Android).
- TeamCode (Android) is reviewed for correctness and structured to compile in an
  FTC SDK project; **explicitly not claimed compiled/verified here** (no SDK).
- `PROGRESS.md` tracks, per phase: built / JVM-tested-green / compile-only /
  "must verify on the robot".

## Working agreement

- Single designated branch `claude/peaceful-cray-5mpak` (harness rule supersedes
  the brief's per-phase branches); small, clear commits per phase.
- Prefer new files in `pedro-extensions`/`TeamCode` over editing Pedro; no forking.
- No destructive git ops without asking.
- If a design assumption breaks against real source, stop and surface options.
