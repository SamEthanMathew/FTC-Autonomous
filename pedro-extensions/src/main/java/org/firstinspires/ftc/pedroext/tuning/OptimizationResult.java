package org.firstinspires.ftc.pedroext.tuning;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * The outcome of an optimization: the best gains found, their cost, and the full
 * trial history (each candidate + its cost) so a run can be inspected or resumed.
 */
public final class OptimizationResult {

    /** One evaluated candidate. */
    public static final class Trial {
        public final GainSet gains;
        public final double cost;

        public Trial(GainSet gains, double cost) {
            this.gains = gains;
            this.cost = cost;
        }
    }

    private final GainSet bestGains;
    private final double bestCost;
    private final List<Trial> history;

    public OptimizationResult(GainSet bestGains, double bestCost, List<Trial> history) {
        this.bestGains = bestGains;
        this.bestCost = bestCost;
        this.history = Collections.unmodifiableList(new ArrayList<>(history));
    }

    /** Builds a result whose best is the minimum-cost trial in the history. */
    public static OptimizationResult fromHistory(List<Trial> history) {
        Trial best = null;
        for (Trial t : history) {
            if (best == null || t.cost < best.cost) {
                best = t;
            }
        }
        if (best == null) {
            throw new IllegalArgumentException("empty optimization history");
        }
        return new OptimizationResult(best.gains, best.cost, history);
    }

    public GainSet getBestGains() { return bestGains; }
    public double getBestCost() { return bestCost; }
    public int getEvaluations() { return history.size(); }
    public List<Trial> getHistory() { return history; }
}
