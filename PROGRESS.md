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

## Phase A — Voltage compensation
- ⬜ (Pedro 2.1.2 already compensates; we expose + test + document.)

## Phase B — Offline simulator
- ⬜

## Phase C — Self-calibration + auto-PID
- ⬜

## Phase D — Logging + replay
- ⬜

## Phase E — Vision-fused localization
- ⬜

## "I must verify on the robot" (accumulating)
- _(none yet)_
