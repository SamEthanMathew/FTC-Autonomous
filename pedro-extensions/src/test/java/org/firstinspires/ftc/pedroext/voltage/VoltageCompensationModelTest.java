package org.firstinspires.ftc.pedroext.voltage;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class VoltageCompensationModelTest {

    private static final double EPS = 1e-9;

    // --- Simple ratio (k = 0): m(V) = nominal / V -----------------------------

    @Test
    void simpleRatioMatchesNominalOverMeasured() {
        VoltageCompensationModel m = VoltageCompensationModel.simpleRatio(12.0);
        assertEquals(1.0, m.multiplier(12.0), EPS);          // at nominal: pass-through
        assertEquals(1.5, m.multiplier(8.0), EPS);           // low pack: boost
        assertEquals(12.0 / 14.0, m.multiplier(14.0), EPS);  // above nominal: cut
        assertEquals(0.5, m.multiplier(24.0), EPS);
    }

    @ParameterizedTest
    @ValueSource(doubles = {6.0, 7.0, 9.5, 12.0, 13.3, 18.0})
    void simpleRatioEqualsTextbookRatioWithinCap(double v) {
        VoltageCompensationModel m = VoltageCompensationModel.simpleRatio(12.0);
        double expected = Math.min(12.0 / v, VoltageCompensationModel.DEFAULT_MAX_BOOST);
        assertEquals(expected, m.multiplier(v), EPS);
    }

    // --- Pedro static-friction formula ---------------------------------------

    @Test
    void atNominalMultiplierIsExactlyOneRegardlessOfFriction() {
        // num = den = Vnom*(1-k), so m == 1 at V == Vnom for any k.
        assertEquals(1.0, new VoltageCompensationModel(12.0, 0.1, 5.0).multiplier(12.0), EPS);
        assertEquals(1.0, new VoltageCompensationModel(12.0, 0.3, 5.0).multiplier(12.0), EPS);
    }

    @Test
    void frictionFormulaMatchesLibraryMath() {
        VoltageCompensationModel m = new VoltageCompensationModel(12.0, 0.1, 5.0); // high cap to see raw value
        // V=6:  num=10.8, den = 6 - (144/6)*0.1 = 3.6  -> 3.0
        assertEquals(3.0, m.multiplier(6.0), EPS);
        // V=8:  den = 8 - (144/8)*0.1 = 6.2 -> 10.8/6.2
        assertEquals(10.8 / 6.2, m.multiplier(8.0), EPS);
        // V=14: den = 14 - (144/14)*0.1 -> 10.8/den
        double den14 = 14.0 - (144.0 / 14.0) * 0.1;
        assertEquals(10.8 / den14, m.multiplier(14.0), EPS);
    }

    // --- Brief's required edge cases -----------------------------------------

    @ParameterizedTest
    @ValueSource(doubles = {12.0, 12.5, 13.0, 14.0, 16.8})
    void atOrAboveNominalNeverBoosts(double v) {
        // "voltage at/above nominal -> <= pass-through"
        assertTrue(VoltageCompensationModel.pedroDefaults().multiplier(v) <= 1.0 + EPS,
                "multiplier should be <= 1 at/above nominal for V=" + v);
        assertTrue(VoltageCompensationModel.simpleRatio(12.0).multiplier(v) <= 1.0 + EPS);
    }

    @Test
    void brownoutClampsAppliedPowerToUnitRange() {
        // "brownout low voltage -> clamp"
        VoltageCompensationModel m = VoltageCompensationModel.simpleRatio(12.0); // maxBoost 2.0
        assertEquals(1.0, m.apply(0.8, 6.0), EPS);    // 0.8 * 2.0 = 1.6 -> clamp 1.0
        assertEquals(-1.0, m.apply(-0.8, 6.0), EPS);  // -1.6 -> -1.0
        assertEquals(0.6, m.apply(0.4, 8.0), EPS);    // 0.4 * 1.5 = 0.6, no clamp
        assertEquals(0.5, m.apply(0.5, 12.0), EPS);   // pass-through at nominal
    }

    @Test
    void severeBrownoutNeverFlipsSign() {
        // Below the critical voltage the raw formula goes negative; the guard
        // must return a positive boost (capped), never a sign-flip.
        VoltageCompensationModel m = new VoltageCompensationModel(12.0, 0.1, 2.0);
        assertEquals(2.0, m.multiplier(3.0), EPS);  // raw would be -6.0
        assertEquals(2.0, m.multiplier(0.0), EPS);  // non-positive voltage
        assertEquals(2.0, m.multiplier(-5.0), EPS);
        assertEquals(2.0, m.multiplier(Double.NaN), EPS);
        // forward command stays forward
        assertTrue(m.apply(0.5, 3.0) > 0.0);
    }

    @Test
    void multiplierIsCappedAtMaxBoost() {
        VoltageCompensationModel m = new VoltageCompensationModel(12.0, 0.0, 1.5);
        assertEquals(1.5, m.multiplier(6.0), EPS);  // raw 2.0, capped to 1.5
    }

    // --- Wheel-power vector mirror of Mecanum --------------------------------

    @Test
    void applyToWheelPowersRenormalizesLikeMecanum() {
        VoltageCompensationModel m = VoltageCompensationModel.simpleRatio(12.0);
        double[] out = m.applyToWheelPowers(new double[]{0.8, 0.4, -0.8, 0.2}, 6.0, 1.0);
        // *2.0 -> {1.6,0.8,-1.6,0.4}; max 1.6 > 1.0 -> scale 0.625
        assertArrayEquals(new double[]{1.0, 0.5, -1.0, 0.25}, out, EPS);
    }

    @Test
    void applyToWheelPowersHonorsMaxPowerScaling() {
        VoltageCompensationModel m = VoltageCompensationModel.simpleRatio(12.0);
        double[] out = m.applyToWheelPowers(new double[]{0.4, 0.0, 0.0, 0.0}, 8.0, 0.5);
        // *1.5 -> {0.6,...}; max 0.6 > 0.5 -> scale to 0.5
        assertArrayEquals(new double[]{0.5, 0.0, 0.0, 0.0}, out, EPS);
    }

    @Test
    void applyToWheelPowersDoesNotMutateInput() {
        VoltageCompensationModel m = VoltageCompensationModel.simpleRatio(12.0);
        double[] in = {0.3, 0.3, 0.3, 0.3};
        m.applyToWheelPowers(in, 6.0, 1.0);
        assertArrayEquals(new double[]{0.3, 0.3, 0.3, 0.3}, in, EPS);
    }

    // --- Construction validation ---------------------------------------------

    @Test
    void rejectsInvalidConstruction() {
        assertThrows(IllegalArgumentException.class, () -> new VoltageCompensationModel(0.0, 0.1, 2.0));
        assertThrows(IllegalArgumentException.class, () -> new VoltageCompensationModel(-12.0, 0.1, 2.0));
        assertThrows(IllegalArgumentException.class, () -> new VoltageCompensationModel(12.0, -0.1, 2.0));
        assertThrows(IllegalArgumentException.class, () -> new VoltageCompensationModel(12.0, 1.0, 2.0));
        assertThrows(IllegalArgumentException.class, () -> new VoltageCompensationModel(12.0, 0.1, 0.5));
    }
}
