# TeamCode (Android / FTC SDK)

These are the on-robot OpModes, `Constants`, and hardware glue for the Pedro
Pathing tooling. **They are not compiled in the headless dev environment** (no
Android SDK there); everything here is reviewed for correctness and structured
to compile inside a normal FTC SDK project, and every motor/sensor/OpMode entry
point is flagged `// HARDWARE-VALIDATION REQUIRED`.

## How to build / run

This directory is meant to be dropped into (or referenced from) a standard FTC
SDK project — e.g. the Pedro Pathing
[Quickstart](https://github.com/Pedro-Pathing/Quickstart) (FTC SDK 11.1.0).
Steps:

1. Copy `src/main/java/org/firstinspires/ftc/teamcode/...` into your project's
   `TeamCode` module (or add this as a source dir).
2. Ensure these dependencies are on the `TeamCode` module (see `build.gradle`):
   ```gradle
   implementation 'com.pedropathing:ftc:2.1.2'
   implementation 'com.pedropathing:telemetry:1.0.0'
   // the pure-JVM tooling module (voltage math, optimizer, EKF, log schema):
   implementation 'org.firstinspires.ftc:pedro-extensions:0.1.0'
   ```
   Resolve `pedro-extensions` either by `./gradlew :pedro-extensions:publishToMavenLocal`
   from the repo root and adding `mavenLocal()`, or via a Gradle composite build
   (`includeBuild '../pedro-extensions'`).
3. Add the custom Maven repo `https://mymaven.bylazar.com/releases` (for
   `com.pedropathing:telemetry`) if not already present. The `core`/`ftc`
   artifacts are also available on Maven Central.
4. Build the `TeamCode` module from Android Studio / the FTC tooling and deploy
   to the Robot Controller.

## Contents (by phase)

- `pedroPathing/Constants.java` — mecanum + Pinpoint config, **voltage
  compensation enabled** (Phase A).
- _(Phase C)_ `tuning/SelfCalibrationOpMode.java`, `tuning/AutoTuneOpMode.java`.
- _(Phase D)_ logging OpMode glue.
- _(Phase E)_ `localization/` AprilTag-fused localizer glue.
