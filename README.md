# FTC-Autonomous — Pedro Pathing tooling

Clean, **non-forking** additions layered on top of the real
[Pedro Pathing](https://github.com/Pedro-Pathing/PedroPathing) library (v2.1.2):
voltage-normalized characterization, a dynamics-accurate offline simulator that
runs the **unmodified** Pedro follower headlessly, an auto-tuning optimizer,
logging/replay, and vision-fused localization.

See [`docs/PLAN.md`](docs/PLAN.md) for the design and ground-truth API facts, and
[`PROGRESS.md`](PROGRESS.md) for live status (what's tested-green vs
compile-only vs hardware-pending).

## Layout

- **`pedro-extensions/`** — standalone, pure-JVM Gradle module. All
  platform-independent logic + every JUnit test. Builds with only a JDK; depends
  on `com.pedropathing:core` from Maven Central, so tests drive the real follower
  without Android.
- **`TeamCode/`** — Android OpModes, `Constants`, and thin hardware glue
  (created from Phase A onward). Compiled inside an FTC SDK project on a machine
  with the Android SDK; consumes `pedro-extensions` + `com.pedropathing:ftc`.

## Build & test (the JVM logic)

```bash
cd pedro-extensions
./gradlew test
```

## Hardware code

Anything under `TeamCode/` that touches motors, sensors, or an OpMode lifecycle
is marked `// HARDWARE-VALIDATION REQUIRED` and listed in `PROGRESS.md`. It is
**not** claimed compiled or verified in this environment (no Android SDK here).
