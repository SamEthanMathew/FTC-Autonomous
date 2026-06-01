package org.firstinspires.ftc.pedroext.tuning;

import java.util.ArrayList;
import java.util.List;

/**
 * Twiddle (coordinate descent) optimizer. Lightweight and dependency-free, so it
 * can run on-robot as well as offline. Per parameter it tries a step up, then a
 * step down, growing the step on improvement and shrinking it otherwise; the
 * working vector always holds the best gains found so far.
 */
public final class TwiddleOptimizer implements Optimizer {

    private final double stepFraction;     // initial step as a fraction of each param's range
    private final double relativeTolerance; // stop when the largest normalized step is below this
    private final int maxEvaluations;

    public TwiddleOptimizer() {
        this(0.3, 0.02, 45);
    }

    public TwiddleOptimizer(double stepFraction, double relativeTolerance, int maxEvaluations) {
        this.stepFraction = stepFraction;
        this.relativeTolerance = relativeTolerance;
        this.maxEvaluations = maxEvaluations;
    }

    @Override
    public OptimizationResult optimize(GainSet initial, TrajectoryEvaluator evaluator) {
        double[] lower = GainSet.lowerBounds();
        double[] upper = GainSet.upperBounds();
        int n = lower.length;

        double[] p = clamp(initial.toArray(), lower, upper);
        double[] dp = new double[n];
        for (int i = 0; i < n; i++) {
            dp[i] = stepFraction * (upper[i] - lower[i]);
        }

        List<OptimizationResult.Trial> history = new ArrayList<>();
        double bestCost = evalAndRecord(evaluator, p, history);

        while (history.size() < maxEvaluations && normalizedStep(dp, lower, upper) > relativeTolerance) {
            for (int i = 0; i < n && history.size() < maxEvaluations; i++) {
                double original = p[i];

                p[i] = clamp1(original + dp[i], lower[i], upper[i]);
                double cost = evalAndRecord(evaluator, p, history);
                if (cost < bestCost) {
                    bestCost = cost;
                    dp[i] *= 1.1;
                    continue;
                }

                p[i] = clamp1(original - dp[i], lower[i], upper[i]);
                cost = evalAndRecord(evaluator, p, history);
                if (cost < bestCost) {
                    bestCost = cost;
                    dp[i] *= 1.1;
                } else {
                    p[i] = original;     // revert; neither direction helped
                    dp[i] *= 0.9;
                }
            }
        }
        return OptimizationResult.fromHistory(history);
    }

    private static double evalAndRecord(TrajectoryEvaluator evaluator, double[] p,
                                        List<OptimizationResult.Trial> history) {
        GainSet g = GainSet.fromArray(p);
        double cost = evaluator.cost(g);
        history.add(new OptimizationResult.Trial(g, cost));
        return cost;
    }

    private static double normalizedStep(double[] dp, double[] lower, double[] upper) {
        double max = 0.0;
        for (int i = 0; i < dp.length; i++) {
            double range = upper[i] - lower[i];
            if (range > 0) {
                max = Math.max(max, dp[i] / range);
            }
        }
        return max;
    }

    private static double[] clamp(double[] a, double[] lower, double[] upper) {
        double[] out = a.clone();
        for (int i = 0; i < out.length; i++) {
            out[i] = clamp1(out[i], lower[i], upper[i]);
        }
        return out;
    }

    private static double clamp1(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
