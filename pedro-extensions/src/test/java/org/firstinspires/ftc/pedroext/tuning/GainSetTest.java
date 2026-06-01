package org.firstinspires.ftc.pedroext.tuning;

import com.pedropathing.follower.FollowerConstants;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Fast, deterministic tests for the gain representation (no simulation). */
class GainSetTest {

    private static final double EPS = 1e-9;

    @Test
    void arrayRoundTrip() {
        double[] a = {0.2, 0.01, 1.5, 0.05, 0.03, 0.001};
        assertArrayEquals(a, GainSet.fromArray(a).toArray(), EPS);
    }

    @Test
    void toConstantsMapsGainsAndKeepsDefaultsElsewhere() {
        FollowerConstants fc = new GainSet(0.2, 0.01, 1.5, 0.05, 0.03, 0.001).toConstants();
        assertEquals(0.2, fc.coefficientsTranslationalPIDF.P, EPS);
        assertEquals(0.01, fc.coefficientsTranslationalPIDF.D, EPS);
        assertEquals(0.015, fc.coefficientsTranslationalPIDF.F, EPS);   // fixed default F
        assertEquals(1.5, fc.coefficientsHeadingPIDF.P, EPS);
        assertEquals(0.05, fc.coefficientsHeadingPIDF.D, EPS);
        assertEquals(0.01, fc.coefficientsHeadingPIDF.F, EPS);
        assertEquals(0.03, fc.coefficientsDrivePIDF.P, EPS);
        assertEquals(0.6, fc.coefficientsDrivePIDF.F, EPS);             // fixed default F
        assertEquals(0.001, fc.centripetalScaling, EPS);
    }

    @Test
    void applyToMutatesExistingConstantsInPlace() {
        FollowerConstants base = new FollowerConstants();
        FollowerConstants returned = new GainSet(0.4, 0.02, 2.0, 0.1, 0.05, 0.002).applyTo(base);
        assertSame(base, returned);                              // same instance (for rebuild-and-retest)
        assertEquals(0.4, base.coefficientsTranslationalPIDF.P, EPS);
        assertEquals(2.0, base.coefficientsHeadingPIDF.P, EPS);
        assertEquals(0.05, base.coefficientsDrivePIDF.P, EPS);
        assertEquals(0.002, base.centripetalScaling, EPS);
    }

    @Test
    void clampToBoundsKeepsWithinRange() {
        double[] lower = GainSet.lowerBounds();
        double[] upper = GainSet.upperBounds();
        GainSet clamped = new GainSet(99, 99, 99, 99, 99, 99).clampedToBounds();
        assertArrayEquals(upper, clamped.toArray(), EPS);
        GainSet clampedLow = new GainSet(-5, -5, -5, -5, -5, -5).clampedToBounds();
        assertArrayEquals(lower, clampedLow.toArray(), EPS);
    }

    @Test
    void boundsAreOrderedAndSameLengthAsParameters() {
        double[] lower = GainSet.lowerBounds();
        double[] upper = GainSet.upperBounds();
        assertEquals(GainSet.NAMES.length, lower.length);
        assertEquals(GainSet.NAMES.length, upper.length);
        for (int i = 0; i < lower.length; i++) {
            assertTrue(upper[i] > lower[i], "upper>lower for " + GainSet.NAMES[i]);
        }
    }
}
