package org.firstinspires.ftc.pedroext.logging;

import java.util.HashMap;
import java.util.Map;

/**
 * One control-loop record for logging/replay, with a fixed, named set of
 * double channels. The <b>same</b> schema is produced by the offline simulator
 * and the on-robot logger, so a predicted (sim) run and an actual (robot) run can
 * be loaded side by side and compared directly (the Phase B ↔ D pairing).
 *
 * <p>Channels capture what the brief asks for: pose, target, the four follower
 * correction vectors, commanded vs. applied (voltage-compensated) power, and
 * battery voltage.
 */
public final class LogFrame {

    /** Ordered channel names; writers iterate these. */
    public static final String[] CHANNELS = {
            "t",
            "poseX", "poseY", "poseHeading",
            "targetX", "targetY", "targetHeading",
            "trackingError", "headingError",
            "correctiveX", "correctiveY",
            "headingVecX", "headingVecY",
            "driveX", "driveY",
            "centripetalX", "centripetalY",
            "commandedPower", "appliedPower",
            "batteryVoltage",
    };

    private static final Map<String, Integer> INDEX = new HashMap<>();
    static {
        for (int i = 0; i < CHANNELS.length; i++) {
            INDEX.put(CHANNELS[i], i);
        }
    }

    private final double[] values;

    private LogFrame(double[] values) {
        this.values = values;
    }

    public static int channelIndex(String name) {
        Integer i = INDEX.get(name);
        if (i == null) {
            throw new IllegalArgumentException("unknown channel: " + name);
        }
        return i;
    }

    /** Timestamp in seconds (the {@code "t"} channel). */
    public double time() {
        return values[0];
    }

    public double get(int channel) {
        return values[channel];
    }

    public double get(String channel) {
        return values[channelIndex(channel)];
    }

    /** Channel values in {@link #CHANNELS} order. */
    public double[] values() {
        return values.clone();
    }

    public static Builder builder() {
        return new Builder();
    }

    public static final class Builder {
        private final double[] values = new double[CHANNELS.length];

        public Builder set(String channel, double value) {
            values[channelIndex(channel)] = value;
            return this;
        }

        public LogFrame build() {
            return new LogFrame(values.clone());
        }
    }
}
