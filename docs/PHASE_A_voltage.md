# Phase A — Voltage compensation

## What and why
Battery voltage sags as the pack drains, so the same commanded power produces
less force late in a match than at full charge — wrecking the repeatability that
characterized dynamics (Phase B/C) rely on. Compensation normalizes commanded
power by measured bus voltage so behavior is battery-independent, and lets us
store dynamics constants in **voltage-normalized units**.

## Key finding: Pedro 2.1.2 already does this
The `Drivetrain` base class (core) carries `useVoltageCompensation` /
`nominalVoltage`, and `Mecanum`/`Swerve` (ftc) apply the multiplier
`m(V) = (Vnom − Vnom·k) / (V − (Vnom²/V)·k)` to wheel powers, then proportionally
renormalize to `maxPowerScaling`. Default is **off**. So per the brief we
**expose, test, and document** rather than reimplement.

## Deliverables
- ✅ `pedro-extensions/.../voltage/VoltageCompensationModel.java` — pure-Java port
  of Pedro's exact multiplier + the per-element clamp to [−1, 1] and the Mecanum
  vector renormalization, so the math is JVM-unit-testable and reusable by the
  offline simulator/logger. Adds a **brownout safety guard**: below the critical
  voltage Pedro's raw formula returns a *negative* multiplier (would reverse
  motor direction); the port clamps to `(0, maxBoost]`.
  - `k = 0` mode reproduces the brief's simple ratio `m(V) = Vnom/V`.
- ✅ `VoltageCompensationModelTest` (JVM, green): known (commanded, V) pairs →
  expected clamped output; V ≥ nominal ⇒ ≤ pass-through; brownout ⇒ clamp;
  `k = 0` ≡ textbook ratio; severe-brownout sign-flip guard; Mecanum-style
  renormalization; construction validation.
- 🟡 `TeamCode/.../pedroPathing/Constants.java` — `useVoltageCompensation(true)`,
  `nominalVoltage(12.0)`, `staticFrictionCoefficient(0.1)` on `MecanumConstants`.
  Compile-elsewhere (no Android SDK here).

## Model notes / divergence from reality
- The multiplier assumes torque ∝ voltage and a constant static-friction term.
  Real motors also have back-EMF and thermal effects not modeled here.
- `maxBoost` (default 2.0) bounds compensation so a deep brownout can't command
  a runaway/!reversed output; tune per robot.

## 🔧 I must verify on the robot
- [ ] Straight-line distance repeatability at full charge vs a drained pack
      (with comp on) — should converge vs the uncompensated spread.
- [ ] Confirm a `VoltageSensor` is present and `getVoltage()` is sane on the bus.
- [ ] Sanity-check `staticFrictionCoefficient` for this drivetrain (default 0.1).
