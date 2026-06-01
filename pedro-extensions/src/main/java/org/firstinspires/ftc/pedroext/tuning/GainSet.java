package org.firstinspires.ftc.pedroext.tuning;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.FollowerConstants;

import java.util.Arrays;

/**
 * The tunable follower gains the auto-PID optimizer searches over: the
 * translational and heading PIDF P/D terms, the drive PIDF P term, and the
 * centripetal scaling. Feedforward (F) and integral (I) terms, the drive D/filter,
 * and all dynamics constants are held at Pedro's defaults so the optimizer tunes
 * exactly the coefficients Pedro exposes (per the brief), and so a candidate maps
 * cleanly onto a {@link FollowerConstants}.
 *
 * <p>Immutable; convertible to/from a plain {@code double[]} for the optimizers.
 */
public final class GainSet {

    // Fixed (not tuned) — Pedro 2.1.2 defaults.
    private static final double TRANSLATIONAL_F = 0.015;
    private static final double HEADING_F = 0.01;
    private static final double DRIVE_D = 0.00001;
    private static final double DRIVE_FILTER = 0.01;
    private static final double DRIVE_F = 0.6;

    public static final String[] NAMES = {
            "translationalP", "translationalD", "headingP", "headingD", "driveP", "centripetalScaling"
    };
    // Search bounds, in NAMES order. Kept positive and physically sane.
    private static final double[] LOWER = {0.01, 0.0, 0.1, 0.0, 0.005, 0.0};
    private static final double[] UPPER = {1.0, 0.05, 5.0, 0.5, 0.2, 0.005};

    private final double translationalP;
    private final double translationalD;
    private final double headingP;
    private final double headingD;
    private final double driveP;
    private final double centripetalScaling;

    public GainSet(double translationalP, double translationalD, double headingP, double headingD,
                   double driveP, double centripetalScaling) {
        this.translationalP = translationalP;
        this.translationalD = translationalD;
        this.headingP = headingP;
        this.headingD = headingD;
        this.driveP = driveP;
        this.centripetalScaling = centripetalScaling;
    }

    /** Pedro 2.1.2 default gains (a strong starting point / reference). */
    public static GainSet pedroDefaults() {
        return new GainSet(0.1, 0.0, 1.0, 0.0, 0.025, 0.0005);
    }

    public static GainSet fromArray(double[] a) {
        return new GainSet(a[0], a[1], a[2], a[3], a[4], a[5]);
    }

    public double[] toArray() {
        return new double[]{translationalP, translationalD, headingP, headingD, driveP, centripetalScaling};
    }

    public static double[] lowerBounds() {
        return LOWER.clone();
    }

    public static double[] upperBounds() {
        return UPPER.clone();
    }

    /** Clamps each parameter into its search bounds. */
    public GainSet clampedToBounds() {
        double[] a = toArray();
        for (int i = 0; i < a.length; i++) {
            a[i] = Math.max(LOWER[i], Math.min(UPPER[i], a[i]));
        }
        return fromArray(a);
    }

    /** Builds a {@link FollowerConstants} with these gains and Pedro defaults elsewhere. */
    public FollowerConstants toConstants() {
        return applyTo(new FollowerConstants());
    }

    /**
     * Writes these gains into an existing {@link FollowerConstants} (mutating its
     * PIDF coefficient holders and centripetal scaling in place), preserving the
     * team's other settings (mass, zero-power accelerations, hardware). Used by the
     * on-robot auto-tuner: mutate the base constants, rebuild the follower, retest.
     *
     * @return the same {@code constants} instance, for chaining
     */
    public FollowerConstants applyTo(FollowerConstants constants) {
        constants.coefficientsTranslationalPIDF.setCoefficients(translationalP, 0.0, translationalD, TRANSLATIONAL_F);
        constants.coefficientsHeadingPIDF.setCoefficients(headingP, 0.0, headingD, HEADING_F);
        constants.coefficientsDrivePIDF.setCoefficients(driveP, 0.0, DRIVE_D, DRIVE_FILTER, DRIVE_F);
        constants.centripetalScaling(centripetalScaling);
        return constants;
    }

    public double getTranslationalP() { return translationalP; }
    public double getTranslationalD() { return translationalD; }
    public double getHeadingP() { return headingP; }
    public double getHeadingD() { return headingD; }
    public double getDriveP() { return driveP; }
    public double getCentripetalScaling() { return centripetalScaling; }

    @Override
    public String toString() {
        return "GainSet" + Arrays.toString(toArray());
    }
}
