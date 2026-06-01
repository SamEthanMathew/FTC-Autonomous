package org.firstinspires.ftc.pedroext.tuning;

import org.apache.commons.math3.analysis.MultivariateFunction;
import org.apache.commons.math3.exception.TooManyEvaluationsException;
import org.apache.commons.math3.optim.InitialGuess;
import org.apache.commons.math3.optim.MaxEval;
import org.apache.commons.math3.optim.SimpleBounds;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.apache.commons.math3.optim.nonlinear.scalar.ObjectiveFunction;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.CMAESOptimizer;
import org.apache.commons.math3.random.MersenneTwister;

import java.util.ArrayList;
import java.util.List;

/**
 * CMA-ES optimizer (Apache Commons Math {@link CMAESOptimizer}) for offline gain
 * tuning, where evaluations are free. Better than coordinate descent at escaping
 * local minima and handling parameter coupling, at the cost of more evaluations —
 * which is why it is the offline/simulator tool while {@link TwiddleOptimizer}
 * remains the lightweight on-robot option.
 *
 * <p>Seeded via a {@link MersenneTwister} so runs are reproducible. The evaluation
 * budget is enforced; if CMA-ES hits it mid-generation we still return the best
 * candidate seen.
 */
public final class CmaesGainOptimizer implements Optimizer {

    private final int populationSize;
    private final int maxEvaluations;
    private final double sigmaFraction;
    private final long seed;

    public CmaesGainOptimizer(long seed) {
        this(10, 60, 0.3, seed);
    }

    public CmaesGainOptimizer(int populationSize, int maxEvaluations, double sigmaFraction, long seed) {
        this.populationSize = populationSize;
        this.maxEvaluations = maxEvaluations;
        this.sigmaFraction = sigmaFraction;
        this.seed = seed;
    }

    @Override
    public OptimizationResult optimize(GainSet initial, TrajectoryEvaluator evaluator) {
        double[] lower = GainSet.lowerBounds();
        double[] upper = GainSet.upperBounds();
        int n = lower.length;

        double[] start = clamp(initial.toArray(), lower, upper);
        double[] sigma = new double[n];
        for (int i = 0; i < n; i++) {
            sigma[i] = sigmaFraction * (upper[i] - lower[i]);
        }

        List<OptimizationResult.Trial> history = new ArrayList<>();
        MultivariateFunction objective = point -> {
            GainSet g = GainSet.fromArray(point);
            double cost = evaluator.cost(g);
            history.add(new OptimizationResult.Trial(g, cost));
            return cost;
        };

        CMAESOptimizer optimizer = new CMAESOptimizer(
                /* maxIterations */ 1000,
                /* stopFitness   */ 0.0,
                /* isActiveCMA   */ true,
                /* diagonalOnly  */ 0,
                /* checkFeasable */ 1,
                new MersenneTwister(seed),
                /* generateStats */ false,
                /* checker       */ null);

        try {
            optimizer.optimize(
                    new MaxEval(maxEvaluations),
                    new ObjectiveFunction(objective),
                    GoalType.MINIMIZE,
                    new InitialGuess(start),
                    new CMAESOptimizer.Sigma(sigma),
                    new CMAESOptimizer.PopulationSize(populationSize),
                    new SimpleBounds(lower, upper));
        } catch (TooManyEvaluationsException ignored) {
            // Budget exhausted mid-generation; the best candidate is in history.
        }
        return OptimizationResult.fromHistory(history);
    }

    private static double[] clamp(double[] a, double[] lower, double[] upper) {
        double[] out = a.clone();
        for (int i = 0; i < out.length; i++) {
            out[i] = Math.max(lower[i], Math.min(upper[i], out[i]));
        }
        return out;
    }
}
