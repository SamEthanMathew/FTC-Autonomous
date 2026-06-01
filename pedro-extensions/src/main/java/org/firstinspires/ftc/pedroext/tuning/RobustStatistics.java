package org.firstinspires.ftc.pedroext.tuning;

import java.util.Arrays;

/**
 * Outlier-robust statistics for the self-calibration harness: median/MAD-based
 * outlier rejection and a guard for the classic "velocity never reached terminal"
 * trap (measurements still climbing across runs). Pure logic, unit-tested.
 */
public final class RobustStatistics {

    /** Scales MAD to a consistent estimator of the standard deviation for normal data. */
    private static final double MAD_TO_SIGMA = 1.4826;

    private RobustStatistics() {
    }

    /** Median of the values (average of the two middle values for even counts). */
    public static double median(double[] values) {
        if (values.length == 0) {
            throw new IllegalArgumentException("no values");
        }
        double[] sorted = values.clone();
        Arrays.sort(sorted);
        int n = sorted.length;
        if ((n & 1) == 1) {
            return sorted[n / 2];
        }
        return 0.5 * (sorted[n / 2 - 1] + sorted[n / 2]);
    }

    public static double mean(double[] values) {
        double sum = 0.0;
        for (double v : values) {
            sum += v;
        }
        return sum / values.length;
    }

    /**
     * Mean after discarding outliers more than {@code madMultiplier} median-absolute-
     * deviations from the median (a robust replacement for "drop the min/max").
     * Falls back to the plain mean when the spread is degenerate.
     *
     * @param madMultiplier typically ~2.5
     */
    public static double robustMean(double[] values, double madMultiplier) {
        if (values.length == 0) {
            throw new IllegalArgumentException("no values");
        }
        if (values.length <= 2) {
            return mean(values);
        }
        double median = median(values);
        double[] deviations = new double[values.length];
        for (int i = 0; i < values.length; i++) {
            deviations[i] = Math.abs(values[i] - median);
        }
        double mad = median(deviations);
        if (mad <= 0.0) {
            return mean(values); // values clustered/identical at the median
        }
        double threshold = madMultiplier * MAD_TO_SIGMA * mad;
        double sum = 0.0;
        int kept = 0;
        for (double v : values) {
            if (Math.abs(v - median) <= threshold) {
                sum += v;
                kept++;
            }
        }
        return kept == 0 ? median : sum / kept;
    }

    /** How many values would be rejected as outliers by {@link #robustMean}. */
    public static int outlierCount(double[] values, double madMultiplier) {
        if (values.length <= 2) {
            return 0;
        }
        double median = median(values);
        double[] deviations = new double[values.length];
        for (int i = 0; i < values.length; i++) {
            deviations[i] = Math.abs(values[i] - median);
        }
        double mad = median(deviations);
        if (mad <= 0.0) {
            return 0;
        }
        double threshold = madMultiplier * MAD_TO_SIGMA * mad;
        int rejected = 0;
        for (double v : values) {
            if (Math.abs(v - median) > threshold) {
                rejected++;
            }
        }
        return rejected;
    }

    /**
     * Detects the accuracy trap where a velocity measurement is still climbing across
     * runs (the robot never reached terminal velocity), by comparing the second half
     * of the run sequence to the first.
     *
     * @param runOrderedSamples measurements in the order runs were taken
     * @param relativeIncreaseThreshold e.g. 0.05 for "5% higher in the second half"
     * @return true if the trend is still climbing beyond the threshold
     */
    public static boolean isStillClimbing(double[] runOrderedSamples, double relativeIncreaseThreshold) {
        int n = runOrderedSamples.length;
        if (n < 2) {
            return false;
        }
        int half = n / 2;
        double firstHalf = mean(Arrays.copyOfRange(runOrderedSamples, 0, Math.max(1, half)));
        double secondHalf = mean(Arrays.copyOfRange(runOrderedSamples, n - Math.max(1, half), n));
        if (firstHalf <= 0.0) {
            return secondHalf > 0.0;
        }
        return (secondHalf - firstHalf) / firstHalf > relativeIncreaseThreshold;
    }
}
