package org.firstinspires.ftc.pedroext.tuning;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * The headline Phase C test: from a deliberately bad gain set, the optimizers
 * drive the <b>real</b> Pedro follower (against the Phase B simulator) to a much
 * lower tracking cost, and CMA-ES does so stably across random seeds. This proves
 * the auto-tuner works before any robot run.
 *
 * <p>These run the real follower in (governed) real time, so they take tens of
 * seconds; the per-evaluation window is short and the budgets are bounded.
 */
class AutoTuneOptimizerTest {

    // A deliberately bad starting point: almost no correction authority and very
    // low drive power (tracks loosely and crawls).
    private static final GainSet BAD_START = new GainSet(0.01, 0.0, 0.1, 0.0, 0.005, 0.0);

    private SimTrajectoryEvaluator evaluator() {
        return new SimTrajectoryEvaluator(ValidationScenario.tuningCurve());
    }

    @Test
    void twiddleReducesTrackingCostFromBadStart() {
        SimTrajectoryEvaluator evaluator = evaluator();
        double badCost = evaluator.cost(BAD_START);

        OptimizationResult result = new TwiddleOptimizer(0.3, 0.04, 20).optimize(BAD_START, evaluator);

        assertTrue(result.getBestCost() < 0.5 * badCost,
                "twiddle should at least halve the cost (bad=" + badCost + ", best=" + result.getBestCost() + ")");
        assertTrue(result.getBestCost() < 2.0,
                "twiddle should reach a good absolute cost, was " + result.getBestCost());
    }

    @Test
    void cmaesReducesCostAndIsStableAcrossSeeds() {
        SimTrajectoryEvaluator evaluator = evaluator();
        double badCost = evaluator.cost(BAD_START);

        OptimizationResult seedA = new CmaesGainOptimizer(8, 18, 0.3, 1L).optimize(BAD_START, evaluator);
        OptimizationResult seedB = new CmaesGainOptimizer(8, 18, 0.3, 7L).optimize(BAD_START, evaluator);

        assertTrue(seedA.getBestCost() < 0.5 * badCost,
                "cmaes(seedA) should at least halve the cost, was " + seedA.getBestCost());
        assertTrue(seedB.getBestCost() < 0.5 * badCost,
                "cmaes(seedB) should at least halve the cost, was " + seedB.getBestCost());
        assertTrue(Math.abs(seedA.getBestCost() - seedB.getBestCost()) < 0.6,
                "cmaes should be stable across seeds: " + seedA.getBestCost() + " vs " + seedB.getBestCost());
    }
}
