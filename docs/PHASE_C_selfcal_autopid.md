# Phase C — Self-calibration + auto-PID

Built C2 (the JVM-testable optimizer) first, validated in the simulator, then the
on-robot OpModes.

## C2 — closed-loop auto-PID optimizer (JVM, the headline)
- `TrajectoryEvaluator` — the one seam (§4): candidate `GainSet` → scalar cost.
  Implemented by `SimTrajectoryEvaluator` (offline, runs the real follower against
  the Phase B sim) and by an on-robot evaluator in TeamCode.
- `GainSet` — the tuned coefficients (translational + heading P/D, drive P,
  centripetal scaling), with bounds; `toConstants()` / `applyTo()` map onto Pedro's
  exact coefficient holders.
- Optimizers behind `Optimizer`: `TwiddleOptimizer` (coordinate descent,
  lightweight enough for the robot) and `CmaesGainOptimizer` (Apache Commons Math
  CMA-ES, for free offline compute).
- **Cost** = tracking RMSE + a progress term `(1 − pathCompletion)` + an overshoot
  term. The progress term is essential: pure RMSE is degenerate (it rewards
  crawling slowly to stay near the path). Evaluated over a fixed-time window so
  every trial costs the same and the surface has no timeout cliffs.

### JVM tests (all green)
- `OptimizerLogicTest` — Twiddle and CMA-ES both minimize a synthetic bowl
  (isolates optimizer correctness from the sim).
- `AutoTuneOptimizerTest` (headline) — from a deliberately bad gain set, **both
  optimizers cut the simulated tracking cost by >50%** (≈ 4.8 → 0.8), and CMA-ES is
  stable across seeds. Proves the tuner works before any robot run.
- `GainSetTest`, `OptimizationResultTest` — fast unit coverage.

## C1 — one-shot self-calibration harness
- JVM-tested logic: `RobustStatistics` (median/MAD outlier rejection with the
  1.4826 σ-scaling, and a "velocity still climbing across runs" guard for the
  classic terminal-velocity trap), `CalibrationResult` + `CalibrationStore`
  (properties I/O + a ready-to-paste constants snippet — no hand-transcribing).
  Covered by `CalibrationTest` (green).
- 🟡 `TeamCode/.../tuning/SelfCalibrationOpMode.java` (compile-elsewhere) — runs
  forward/lateral velocity + forward/lateral zero-power-accel measurements N times
  (alternating direction), rejects outliers, averages, writes results to device
  storage. Guard rails: minimum distance, climbing warning. Based on the
  Quickstart tuners' measurement method.

## C2 on-robot
- 🟡 `TeamCode/.../tuning/AutoTuneOpMode.java` (compile-elsewhere) — the SAME
  `TwiddleOptimizer` behind an on-robot `TrajectoryEvaluator` that rebuilds the
  follower per candidate, drives a there-and-back path, scores RMSE + endpoint
  error, and logs every trial's gains + cost to CSV (resume/inspect).

## 🔧 I must verify on the robot
- [ ] Self-cal velocity/zero-power measurement math on hardware; confirm written
      `pedro_calibration.properties` values are sane vs manual tuners.
- [ ] Auto-tune: the there-and-back path returns the robot to start each trial on
      YOUR field; supervise; confirm tuned gains behave (sim-optimal gains can be
      too aggressive on a real robot — re-validate, and re-tune on-robot if needed).
- [ ] Confirm `applyTo` + rebuild actually changes follower behavior per trial.
