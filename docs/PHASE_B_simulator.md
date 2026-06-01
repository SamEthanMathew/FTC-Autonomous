# Phase B — Dynamics-accurate offline simulator

## What and why
The auto-tuner (Phase C) needs to be validated without a robot, and we want to
compare predicted vs. actual traces (Phase D). This phase runs the **unmodified**
Pedro follower headlessly against a simulated plant, entirely on the JVM.

## How it works
- `RobotPlant` — discrete-time plant. Translational velocity is **slew-rate
  limited**: accelerates at `maxAcceleration`, decelerates at the team's
  **measured zero-power (coast) acceleration** as a *constant* rate. Heading is
  first-order. Added mass scales the effective accelerations. Pose integrates in
  the field frame.
- `SimulatedDrivetrain` (extends core `Drivetrain`) decodes the follower's
  corrective/heading/pathing vectors into a robot-frame velocity command — net
  translation = `corrective + pathing` (rotated to robot frame, scaled by
  `xVelocity`/`yVelocity`); signed yaw = projection of the heading vector onto
  the heading direction (positive = CCW).
- `SimulatedLocalizer` (implements core `Localizer`) reports the plant pose, with
  optional per-loop Gaussian sensor noise.
- `SimulatedRobot` wires these to a real `Follower`; `SimulationLoop` runs it;
  `ScheduledDisturbance` injects kicks/added mass; `SimulationResult`/`TraceSample`
  carry metrics + a CSV trace dump.

## Critical design constraint: governed real time
Pedro's `PIDFController`, `FilteredPIDFController`, and `PoseTracker` read
`System.nanoTime()` directly (no injectable clock), so their derivative/integral
and velocity terms use wall-clock dt. A naive tight loop would feed microsecond
dt and destabilize them. The loop therefore **paces to a target period and
integrates the plant by the *measured* wall dt**, keeping plant and follower
clocks consistent even under jitter. Consequence: sim time ≈ wall time, so tests
use short paths.

## Why slew-rate (constant decel), not exponential first-order
An exponential relaxation makes deceleration proportional to speed (viscous),
but Pedro characterizes — and its predictive braking assumes — a roughly
*constant* coast deceleration. With an exponential model the robot decelerated
far too gently at low speed and **overshot every endpoint by ~4–6 in**. Using
the measured zero-power acceleration as a constant decel rate makes the
follower's braking line up with the plant; endpoints converge to < 1–2 in. (This
was caught by the convergence tests and the trace dump.)

## JVM tests (all green)
- `RobotPlantTest` (9): accel/decel rate caps, coast decel == measured rate,
  steady-state velocity, motion along heading, added-mass slowdown, position
  kick, angle wrap, dt guard.
- `SimulatorFollowerTest` (6): real follower converges on straight, strafe,
  curved Bézier, and multi-segment chain paths within tolerance; recovers from a
  mid-path position kick; emits an inspectable CSV trace dump.

## Model assumptions / where it diverges from reality
First-order velocity approximation. Does NOT model wheel slip, motor
back-EMF/torque curves, uneven weight distribution, friction beyond two axes, the
heading-vs-translation saturation trade-off Pedro makes at full power, or battery
sag (handled by the Phase A voltage model). Diverges most in hard-saturation,
high-slip, and very-sharp-acceleration regimes. Acceleration defaults to 2× the
measured coast deceleration (configurable) since there is no measured accel
constant.

## 🔧 I must verify on the robot
- [ ] Compare a real logged run (Phase D) against the sim trace for the same path
      and tune `maxAcceleration` / heading time constant to match reality.
- [ ] Confirm the measured zero-power accelerations used by the plant are current
      (Phase C self-cal).
