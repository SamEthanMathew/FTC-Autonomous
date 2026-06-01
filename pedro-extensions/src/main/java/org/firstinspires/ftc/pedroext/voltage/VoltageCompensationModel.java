package org.firstinspires.ftc.pedroext.voltage;

/**
 * Bus-voltage normalization of commanded motor power, characterized in
 * voltage-normalized units so dynamics constants are battery-independent.
 *
 * <p>Pedro Pathing 2.1.2 <em>already</em> performs voltage compensation inside its
 * {@code Mecanum}/{@code Swerve} drivetrains (see
 * {@code com.pedropathing.ftc.drivetrains.Mecanum#getVoltageNormalized}). Those
 * classes live in the Android-only {@code ftc} module, so the formula cannot be
 * unit-tested directly on the JVM. This class is a faithful, pure-Java port of
 * the exact same math so it can be (a) unit-tested here and (b) reused by offline
 * tooling (the simulator and the logger) without pulling in Android.
 *
 * <p>The multiplier is
 * <pre>
 *   m(V) = (Vnom - Vnom*k) / (V - (Vnom^2 / V)*k)
 * </pre>
 * where {@code Vnom} is the nominal voltage and {@code k} the static-friction
 * coefficient. With {@code k == 0} this collapses to the textbook ratio
 * {@code m(V) = Vnom / V} (the brief's {@code commanded * NOMINAL/measured}).
 *
 * <p><b>Safety improvement over the raw library formula:</b> below a critical
 * voltage the denominator goes non-positive and the raw formula returns a
 * negative multiplier — which would silently <em>reverse</em> motor direction
 * during a brownout. This port clamps the multiplier to {@code (0, maxBoost]}
 * and treats any non-finite/non-positive result as "maximum help needed"
 * ({@code maxBoost}); the per-power clamp to [-1, 1] then bounds the output.
 *
 * <p>Instances are immutable and thread-safe.
 */
public final class VoltageCompensationModel {

    /** Nominal FTC battery voltage; the brief's default. */
    public static final double DEFAULT_NOMINAL_VOLTAGE = 12.0;
    /** Default static-friction coefficient — matches Pedro {@code MecanumConstants}. */
    public static final double DEFAULT_STATIC_FRICTION_COEFFICIENT = 0.1;
    /** Default cap on the multiplier, guarding against brownout sign-flips / blow-ups. */
    public static final double DEFAULT_MAX_BOOST = 2.0;

    private final double nominalVoltage;
    private final double staticFrictionCoefficient;
    private final double maxBoost;

    /**
     * @param nominalVoltage            nominal bus voltage, &gt; 0 (e.g. 12.0)
     * @param staticFrictionCoefficient {@code k} in [0, 1); 0 gives the simple ratio
     * @param maxBoost                  cap on the multiplier, &ge; 1
     */
    public VoltageCompensationModel(double nominalVoltage, double staticFrictionCoefficient, double maxBoost) {
        if (!(nominalVoltage > 0.0) || !Double.isFinite(nominalVoltage)) {
            throw new IllegalArgumentException("nominalVoltage must be finite and > 0, was " + nominalVoltage);
        }
        if (!(staticFrictionCoefficient >= 0.0) || staticFrictionCoefficient >= 1.0) {
            throw new IllegalArgumentException("staticFrictionCoefficient must be in [0, 1), was " + staticFrictionCoefficient);
        }
        if (!(maxBoost >= 1.0) || !Double.isFinite(maxBoost)) {
            throw new IllegalArgumentException("maxBoost must be finite and >= 1, was " + maxBoost);
        }
        this.nominalVoltage = nominalVoltage;
        this.staticFrictionCoefficient = staticFrictionCoefficient;
        this.maxBoost = maxBoost;
    }

    public VoltageCompensationModel(double nominalVoltage, double staticFrictionCoefficient) {
        this(nominalVoltage, staticFrictionCoefficient, DEFAULT_MAX_BOOST);
    }

    /** The Pedro {@code MecanumConstants} defaults: 12.0 V, k = 0.1. */
    public static VoltageCompensationModel pedroDefaults() {
        return new VoltageCompensationModel(DEFAULT_NOMINAL_VOLTAGE, DEFAULT_STATIC_FRICTION_COEFFICIENT);
    }

    /** The brief's simple ratio model: {@code m(V) = nominal / V} (k = 0). */
    public static VoltageCompensationModel simpleRatio(double nominalVoltage) {
        return new VoltageCompensationModel(nominalVoltage, 0.0);
    }

    /**
     * The normalization multiplier for a measured bus voltage, clamped to
     * {@code (0, maxBoost]}. At/above nominal this is &le; 1 (output reduced);
     * below nominal it is &gt; 1 (output boosted). Non-finite or non-positive raw
     * results (severe brownout) return {@code maxBoost}.
     */
    public double multiplier(double measuredVoltage) {
        if (!(measuredVoltage > 0.0) || !Double.isFinite(measuredVoltage)) {
            return maxBoost;
        }
        double numerator = nominalVoltage - nominalVoltage * staticFrictionCoefficient;
        double denominator = measuredVoltage
                - (nominalVoltage * nominalVoltage / measuredVoltage) * staticFrictionCoefficient;
        double m = numerator / denominator;
        if (!Double.isFinite(m) || m <= 0.0) {
            return maxBoost;
        }
        return Math.min(m, maxBoost);
    }

    /**
     * Normalizes a single commanded power and clamps to [-1, 1].
     *
     * @param commandedPower the controller's commanded power (sign = direction)
     * @param measuredVoltage measured bus voltage
     */
    public double apply(double commandedPower, double measuredVoltage) {
        return clamp(commandedPower * multiplier(measuredVoltage), -1.0, 1.0);
    }

    /**
     * Mirrors Pedro {@code Mecanum}: scales an entire wheel-power vector by the
     * multiplier, then — if the largest magnitude exceeds {@code maxPowerScaling}
     * — proportionally renormalizes the whole vector down to {@code maxPowerScaling}
     * (preserving direction), exactly as the library does.
     *
     * @return a new array; the input is not modified.
     */
    public double[] applyToWheelPowers(double[] commandedPowers, double measuredVoltage, double maxPowerScaling) {
        double m = multiplier(measuredVoltage);
        double[] out = new double[commandedPowers.length];
        double maxAbs = 0.0;
        for (int i = 0; i < commandedPowers.length; i++) {
            out[i] = commandedPowers[i] * m;
            maxAbs = Math.max(maxAbs, Math.abs(out[i]));
        }
        if (maxAbs > maxPowerScaling && maxAbs > 0.0) {
            double scale = maxPowerScaling / maxAbs;
            for (int i = 0; i < out.length; i++) {
                out[i] *= scale;
            }
        }
        return out;
    }

    public double getNominalVoltage() {
        return nominalVoltage;
    }

    public double getStaticFrictionCoefficient() {
        return staticFrictionCoefficient;
    }

    public double getMaxBoost() {
        return maxBoost;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
