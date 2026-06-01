# Phase D — File logging + replay

## What and why
Capture per-loop state to a file AdvantageScope can read, in a schema **shared by
the simulator and the robot**, so a predicted (sim) run and an actual (robot) run
overlay directly — closing the loop with Phase B (validate the model, tune
`maxAcceleration`/heading-tau to match reality).

## Schema (`LogFrame`)
A fixed, named set of double channels: `t`, pose (x/y/heading), target
(x/y/heading), `trackingError`, `headingError`, the four correction vectors
(corrective / heading / drive / centripetal, x+y), `commandedPower`,
`appliedPower` (voltage-compensated), `batteryVoltage`. Both the sim and the robot
emit exactly these columns.

## Writers (JVM, tested)
- `CsvDataLogWriter` — the documented, human/spreadsheet-readable schema.
- `WpilogWriter` — WPILOG 1.0 binary (AdvantageScope-native). Uniform max-width
  records (header byte `0x7F`), control "Start" records define one `double`
  channel each, data records carry the values; little-endian, µs timestamps.
- `LogFrames` — converts simulator `TraceSample`s into `LogFrame`s, so sim runs
  log through the identical pipeline.

## JVM tests (all green)
`LoggingTest`: builder/channel checks; CSV header+rows; **WPILOG full round-trip**
(an embedded decoder parses magic/version/Start records/data records back and
verifies channel values at each timestamp); and a short simulated run logged to
both WPILOG and CSV, decoded back, frame count verified (the B↔D pairing).

## On-robot glue (compile-elsewhere)
- 🟡 `TeamCode/.../logging/FollowerLogger.java` — one `LogFrame` per loop from a
  live follower + `VoltageSensor`; reuses the Phase A `VoltageCompensationModel`
  to log commanded vs. applied power.
- 🟡 `TeamCode/.../logging/LoggingDemoOpMode.java` — follows a path while writing a
  timestamped `.wpilog` to device storage.

## 🔧 I must verify on the robot
- [ ] On-robot file writes succeed and the `.wpilog` opens in AdvantageScope.
- [ ] Overlay a real run on the sim trace for the same path; quantify divergence
      and feed it back into the Phase B model parameters.
- [ ] Confirm commanded-vs-applied power logging matches Pedro's internal
      voltage compensation (or document the difference).
