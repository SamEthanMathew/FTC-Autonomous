package org.firstinspires.ftc.pedroext.tuning;

import org.junit.jupiter.api.Test;

import java.util.Arrays;
import java.util.Collections;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertSame;

/** Fast tests for the optimization result/history bookkeeping. */
class OptimizationResultTest {

    @Test
    void fromHistoryPicksMinimumCost() {
        GainSet a = GainSet.pedroDefaults();
        GainSet b = new GainSet(0.2, 0, 1.5, 0, 0.03, 0.001);
        GainSet c = new GainSet(0.3, 0, 2.0, 0, 0.05, 0.001);
        OptimizationResult r = OptimizationResult.fromHistory(Arrays.asList(
                new OptimizationResult.Trial(a, 5.0),
                new OptimizationResult.Trial(b, 1.2),
                new OptimizationResult.Trial(c, 3.0)));
        assertSame(b, r.getBestGains());
        assertEquals(1.2, r.getBestCost(), 1e-9);
        assertEquals(3, r.getEvaluations());
    }

    @Test
    void fromEmptyHistoryThrows() {
        assertThrows(IllegalArgumentException.class,
                () -> OptimizationResult.fromHistory(Collections.emptyList()));
    }
}
