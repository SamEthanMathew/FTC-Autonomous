package org.firstinspires.ftc.pedroext.tuning;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Fast, deterministic tests of the optimizer algorithms themselves, against a
 * synthetic quadratic bowl with a known minimum (no simulation). This isolates
 * "does the optimizer minimize?" from "does the simulator evaluator work?".
 */
class OptimizerLogicTest {

    /** Range-normalized squared distance to a known target — a clean convex bowl. */
    private static final class BowlEvaluator implements TrajectoryEvaluator {
        private final double[] target;
        private final double[] lower = GainSet.lowerBounds();
        private final double[] upper = GainSet.upperBounds();

        BowlEvaluator(GainSet target) {
            this.target = target.toArray();
        }

        @Override
        public double cost(GainSet gains) {
            double[] g = gains.toArray();
            double sum = 0.0;
            for (int i = 0; i < g.length; i++) {
                double n = (g[i] - target[i]) / (upper[i] - lower[i]);
                sum += n * n;
            }
            return sum;
        }
    }

    private final GainSet target = new GainSet(0.35, 0.02, 1.8, 0.1, 0.06, 0.0015);
    private final GainSet start = new GainSet(0.05, 0.0, 0.3, 0.0, 0.01, 0.004);

    @Test
    void twiddleFindsTheMinimum() {
        BowlEvaluator bowl = new BowlEvaluator(target);
        OptimizationResult r = new TwiddleOptimizer(0.3, 1e-4, 1000).optimize(start, bowl);
        assertTrue(r.getBestCost() < 1e-2, "twiddle should reach the bowl minimum, was " + r.getBestCost());
    }

    @Test
    void cmaesFindsTheMinimum() {
        BowlEvaluator bowl = new BowlEvaluator(target);
        // A separable quadratic is the ideal case for coordinate descent; CMA-ES
        // (built for ill-conditioned/non-separable problems) needs more budget to
        // match that precision, so allow more evaluations and a slightly looser bound.
        OptimizationResult r = new CmaesGainOptimizer(12, 1200, 0.3, 42L).optimize(start, bowl);
        assertTrue(r.getBestCost() < 0.03, "cmaes should reach near the bowl minimum, was " + r.getBestCost());
    }

    @Test
    void optimizersImproveOnTheStartingPoint() {
        BowlEvaluator bowl = new BowlEvaluator(target);
        double startCost = bowl.cost(start);
        assertTrue(new TwiddleOptimizer().optimize(start, bowl).getBestCost() < startCost);
        assertTrue(new CmaesGainOptimizer(42L).optimize(start, bowl).getBestCost() < startCost);
    }
}
