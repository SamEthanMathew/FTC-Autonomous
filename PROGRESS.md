# PROGRESS

Status legend: ✅ JVM-tested & green · 🟡 compile-elsewhere (no Android SDK here) ·
🔧 must verify on the robot · ⬜ not started

See `docs/PLAN.md` for the full plan and the ground-truth API facts.

## §0 assumptions in effect (configurable — say the word to change any)
- TEAMCODE_REPO = this repo (was empty; scaffolded fresh)
- FTC_SDK_VERSION = 11.1.0 (detected)
- Pedro Pathing = **2.1.2** (core + ftc on Maven Central) — brief's "1.1.0" was outdated
- DRIVETRAIN = **mecanum** (swerve extension points noted)
- LOCALIZER_HW = configurable; OpMode examples default to goBILDA **Pinpoint**
- Season for Phase E = **DECODE (2025–26)**; exclude Obelisk tags (21–23)

## Environment
- JDK 21, Gradle 8.14.3 (+ wrapper pinned to 8.9). **No Android SDK** → TeamCode
  OpModes are written/structured but cannot be compiled here.
- Network: Maven Central reachable; `com.pedropathing:core/ftc` + commons-math3 +
  FTC SDK artifacts all resolvable. Outbound git works.

## Foundation
- ✅ `pedro-extensions` standalone JVM Gradle module builds; `./gradlew test` green.
- ✅ De-risk: real Pedro `core` (`Follower`/`Drivetrain`/`Localizer`) resolves from
  Maven Central and loads on the JVM (`PedroCoreResolutionSmokeTest`).

## Phase A — Voltage compensation  (see docs/PHASE_A_voltage.md)
Pedro 2.1.2 already compensates internally, so we exposed + tested + documented.
- ✅ `voltage/VoltageCompensationModel` — pure-Java port of Pedro's exact
  multiplier + [-1,1] clamp + Mecanum-style renormalization + brownout
  sign-flip guard; `k=0` reproduces the brief's `nominal/V` ratio.
- ✅ `VoltageCompensationModelTest` green (18 assertions: edges, brownout, k=0
  equivalence, renormalization, validation).
- 🟡 `TeamCode/.../Constants.java` enables `useVoltageCompensation(true)`,
  `nominalVoltage(12.0)` (compile-elsewhere — no Android SDK here).
- 🔧 Robot: straight-line distance repeatability full vs drained pack.

## Phase B — Offline simulator  (see docs/PHASE_B_simulator.md)
Runs the unmodified Pedro follower headlessly against a simulated plant.
- ✅ `sim/RobotPlant` (slew-rate plant; constant measured-decel braking),
  `SimulatedDrivetrain` (extends core `Drivetrain`), `SimulatedLocalizer`
  (implements core `Localizer`), `SimulatedRobot`, `SimulationLoop` (governed
  real-time; integrates by measured wall-dt because Pedro's PIDF/PoseTracker use
  `System.nanoTime()`), `ScheduledDisturbance`, `TraceSample`, `SimulationResult`
  (+ CSV trace dump).
- ✅ `RobotPlantTest` (9) — dynamics math green.
- ✅ `SimulatorFollowerTest` (6) — real follower converges on straight/strafe/
  curve/multi-segment, recovers from a position kick, emits a trace dump.
- 🔧 Robot: compare a real logged run vs sim trace; tune accel/heading-tau to match.

## Phase C — Self-calibration + auto-PID  (see docs/PHASE_C_selfcal_autopid.md)
- ✅ C2 optimizer (JVM): `TrajectoryEvaluator` seam, `GainSet`, `SimTrajectoryEvaluator`,
  `TwiddleOptimizer` + `CmaesGainOptimizer` (Commons Math). Cost = RMSE +
  (1-pathCompletion) progress term + overshoot.
- ✅ `OptimizerLogicTest` (synthetic bowl) + headline `AutoTuneOptimizerTest`:
  from a bad gain set both optimizers cut simulated cost >50% (≈4.8→0.8), CMA-ES
  stable across seeds.
- ✅ C1 logic (JVM): `RobustStatistics` (MAD outlier rejection + climbing guard),
  `CalibrationResult`/`CalibrationStore` (+ constants snippet). `CalibrationTest` green.
- 🟡 `TeamCode/.../tuning/SelfCalibrationOpMode.java` (C1) and `AutoTuneOpMode.java`
  (C2 on-robot, same optimizer) — compile-elsewhere.
- 🔧 Robot: self-cal measurement math; auto-tune return-to-start + re-validate
  sim-optimal gains on hardware.
- Full pedro-extensions suite: 56 JVM tests green.

## Phase D — Logging + replay  (see docs/PHASE_D_logging.md)
- ✅ `logging/LogFrame` shared schema (pose, target, 4 correction vectors,
  commanded vs applied power, voltage). `CsvDataLogWriter` + `WpilogWriter`
  (WPILOG 1.0 binary for AdvantageScope). `LogFrames` converts sim traces → same schema.
- ✅ `LoggingTest`: CSV; WPILOG full round-trip (embedded decoder); short sim run
  logged to WPILOG+CSV and decoded back (B↔D pairing).
- 🟡 `TeamCode/.../logging/FollowerLogger.java` + `LoggingDemoOpMode.java`
  (compile-elsewhere; reuses Phase A voltage model for commanded-vs-applied).
- 🔧 Robot: on-robot file writes; open .wpilog in AdvantageScope; overlay real vs sim.

## Phase E — Vision-fused localization
- ⬜

## "I must verify on the robot" (accumulating)
- [A] Straight-line distance repeatability, full vs drained pack, comp on.
- [A] Confirm a `VoltageSensor` is present and `getVoltage()` reads sane.
- [B] Compare a real logged run vs sim trace; tune accel/heading-tau to match.
- [C] Self-cal measurement math; sane `pedro_calibration.properties` values.
- [C] Auto-tune return-to-start path on your field; re-validate sim-optimal gains
      on hardware (may be too aggressive); supervise.
