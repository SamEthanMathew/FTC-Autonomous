# Phase E — Vision-fused localization

## What and why
Dead-reckoning odometry drifts; AprilTags give absolute fixes. This phase fuses
them in an EKF so the follower gets a drift-corrected pose — without the follower
changing at all (it just receives a different `Localizer`).

## Design (JVM core, tested)
- `PoseEKF` — 3-DOF (x, y, heading) Kalman filter. Odometry displacement drives
  `predict`; an absolute pose fix drives `update` (identity measurement model;
  heading innovation wrapped). **Smoothness:** the per-update correction is
  magnitude-capped (position + heading), so a large innovation (e.g. the first tag
  after a shove) is applied gradually over loops — the follower never sees a jump.
- `VisionCovariance` — quality → (a) a fix-quality **gate** (min decision margin,
  max range, max bearing) and (b) a **covariance**: trust is highest for a near,
  centered, high-margin tag and degrades with range/angle.
- `TagMap` — **config** of valid localization tag IDs. `decodeDefault()` uses the
  2025–26 goal tags (20, 24) and **excludes the Obelisk motif tags 21–23** (not
  localization tags). Confirm IDs/poses against the Game Manual.
- `VisionFusedLocalizer implements Localizer` — wraps the odometry localizer,
  predicts from its deltas, and applies gated, covariance-weighted, smoothed
  vision updates. Velocity is delegated to odometry.

## JVM tests (all green)
`FusionTest`: EKF predict/update + correction-cap; covariance gate + range/angle
weighting; DECODE tag map excludes Obelisk; and two synthetic-stream integration
tests with known ground truth —
- **bounded drift**: with odometry drifting unboundedly (final error > 3.5 in),
  the fused estimate stays bounded (< 1.5 in);
- **collision recovery + smoothness**: a sudden shove the odometry misses spikes
  the error (> 3 in), then vision recovers it (< 1.5 in) with no per-loop pose
  jump (< 3 in/loop).

## On-robot glue (compile-elsewhere)
- 🟡 `AprilTagVisionSource` — VisionPortal + AprilTagProcessor → `AprilTagObservation`s
  using the SDK's built-in `detection.robotPose`; returns only `TagMap` tags.
- 🟡 `VisionFusedLocalizationOpMode` — wires Pinpoint odometry → `VisionFusedLocalizer`
  → follower via the standard `FollowerBuilder.setLocalizer` seam.

## Swerve note
Localization is drivetrain-agnostic (works against the `Localizer` interface), so
Phase E applies to mecanum and swerve identically.

## 🔧 I must verify on the robot
- [ ] Camera calibration, camera-on-robot pose, and the season tag library.
- [ ] Confirm `detection.robotPose` field coordinates match Pedro's frame (units +
      origin + heading sign); add a transform if not.
- [ ] Confirm valid localization tag IDs/poses for the season (exclude Obelisk).
- [ ] Watch the fused pose stay smooth as tags enter/leave view; tune
      `VisionCovariance` and the correction caps.
