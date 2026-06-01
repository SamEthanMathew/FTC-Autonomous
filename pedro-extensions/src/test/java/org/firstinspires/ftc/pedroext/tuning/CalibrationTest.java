package org.firstinspires.ftc.pedroext.tuning;

import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.io.StringReader;
import java.io.StringWriter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Fast, deterministic tests for the self-calibration statistics + persistence. */
class CalibrationTest {

    private static final double EPS = 1e-9;

    @Test
    void medianOddAndEven() {
        assertEquals(2.0, RobustStatistics.median(new double[]{1, 3, 2}), EPS);
        assertEquals(2.5, RobustStatistics.median(new double[]{1, 2, 3, 4}), EPS);
    }

    @Test
    void robustMeanRejectsOutliers() {
        double[] v = {10.0, 10.1, 9.9, 10.05, 50.0}; // 50 is a bad run
        assertEquals(10.0125, RobustStatistics.robustMean(v, 2.5), 1e-3);
        assertEquals(1, RobustStatistics.outlierCount(v, 2.5));
    }

    @Test
    void robustMeanFallsBackForSmallOrFlatSamples() {
        assertEquals(11.0, RobustStatistics.robustMean(new double[]{10.0, 12.0}, 2.5), EPS); // <=2 -> mean
        assertEquals(7.0, RobustStatistics.robustMean(new double[]{7, 7, 7, 7}, 2.5), EPS);  // mad=0 -> mean
    }

    @Test
    void detectsVelocityStillClimbing() {
        assertTrue(RobustStatistics.isStillClimbing(new double[]{60, 63, 66, 70, 74}, 0.05),
                "monotonically climbing runs => never reached terminal velocity");
        assertFalse(RobustStatistics.isStillClimbing(new double[]{60.0, 60.5, 59.7, 60.2, 60.1}, 0.05),
                "stable runs => fine");
    }

    @Test
    void calibrationResultPropertiesRoundTrip() throws IOException {
        CalibrationResult original = new CalibrationResult(81.3, 65.4, -41.2, -59.7, 5);
        StringWriter writer = new StringWriter();
        CalibrationStore.write(writer, original);
        CalibrationResult loaded = CalibrationStore.read(new StringReader(writer.toString()));
        assertEquals(81.3, loaded.getXVelocity(), EPS);
        assertEquals(65.4, loaded.getYVelocity(), EPS);
        assertEquals(-41.2, loaded.getForwardZeroPowerAcceleration(), EPS);
        assertEquals(-59.7, loaded.getLateralZeroPowerAcceleration(), EPS);
        assertEquals(5, loaded.getRunCount());
    }

    @Test
    void calibrationSnippetContainsMeasuredValues() {
        String snippet = new CalibrationResult(80.0, 64.0, -40.0, -58.0, 5).toConstantsSnippet();
        assertTrue(snippet.contains("xVelocity(80.0)"));
        assertTrue(snippet.contains("forwardZeroPowerAcceleration(-40.0)"));
    }

    @Test
    void readMissingPropertyThrows() {
        assertThrows(IOException.class, () -> CalibrationStore.read(new StringReader("xVelocity=80\n")));
    }
}
