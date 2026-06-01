package org.firstinspires.ftc.pedroext.sim;

import com.pedropathing.geometry.Pose;

import java.io.BufferedWriter;
import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.Collections;
import java.util.List;

/**
 * The outcome of one simulated follow: the per-loop trace plus summary metrics
 * (tracking-error max and RMSE, final pose, duration, timeout flag). RMSE is the
 * basis for the Phase C auto-tuning cost.
 */
public final class SimulationResult {

    private final List<TraceSample> samples;
    private final Pose finalPose;
    private final double durationSeconds;
    private final boolean timedOut;
    private final double maxTrackingError;
    private final double rmseTrackingError;

    public SimulationResult(List<TraceSample> samples, double finalX, double finalY, double finalHeading,
                            double durationSeconds, boolean timedOut) {
        this.samples = Collections.unmodifiableList(samples);
        this.finalPose = new Pose(finalX, finalY, finalHeading);
        this.durationSeconds = durationSeconds;
        this.timedOut = timedOut;

        double max = 0.0;
        double sumSq = 0.0;
        for (TraceSample s : samples) {
            max = Math.max(max, s.trackingError);
            sumSq += s.trackingError * s.trackingError;
        }
        this.maxTrackingError = max;
        this.rmseTrackingError = samples.isEmpty() ? 0.0 : Math.sqrt(sumSq / samples.size());
    }

    public List<TraceSample> getSamples() { return samples; }
    public Pose getFinalPose() { return finalPose; }
    public double getDurationSeconds() { return durationSeconds; }
    public boolean isTimedOut() { return timedOut; }
    public double getMaxTrackingError() { return maxTrackingError; }
    public double getRmseTrackingError() { return rmseTrackingError; }

    /** Path completion (0..1) at the end of the run. */
    public double getFinalPathCompletion() {
        return samples.isEmpty() ? 0.0 : samples.get(samples.size() - 1).pathCompletion;
    }

    /** Simulated time elapsed (sum of integration steps), jitter-free vs wall time. */
    public double getSimTimeSeconds() {
        return samples.isEmpty() ? 0.0 : samples.get(samples.size() - 1).t;
    }

    public double finalPositionErrorTo(Pose target) {
        return Math.hypot(finalPose.getX() - target.getX(), finalPose.getY() - target.getY());
    }

    public double finalHeadingErrorTo(Pose target) {
        return Math.abs(RobotPlant.normalizeAngle(finalPose.getHeading() - target.getHeading()));
    }

    /** Largest tracking error observed after a given time (for recovery assertions). */
    public double maxTrackingErrorAfter(double tSeconds) {
        double max = 0.0;
        for (TraceSample s : samples) {
            if (s.t >= tSeconds) {
                max = Math.max(max, s.trackingError);
            }
        }
        return max;
    }

    /** Writes the full per-loop trace as CSV (the headless trace dump). */
    public void writeCsv(File file) throws IOException {
        File parent = file.getParentFile();
        if (parent != null) {
            Files.createDirectories(parent.toPath());
        }
        try (BufferedWriter w = Files.newBufferedWriter(file.toPath(), StandardCharsets.UTF_8)) {
            w.write(TraceSample.csvHeader());
            w.newLine();
            for (TraceSample s : samples) {
                w.write(s.toCsvRow());
                w.newLine();
            }
        }
    }
}
