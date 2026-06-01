package org.firstinspires.ftc.pedroext.tuning;

import java.io.IOException;
import java.io.Reader;
import java.io.Writer;
import java.util.Properties;

/**
 * Reads/writes a {@link CalibrationResult} as a properties stream. The on-robot
 * OpMode writes one to device storage so results flow into the constants without
 * hand-transcribing; tests and tools can load it back. Pure I/O, unit-tested.
 */
public final class CalibrationStore {

    private static final String X_VELOCITY = "xVelocity";
    private static final String Y_VELOCITY = "yVelocity";
    private static final String FORWARD_ZPA = "forwardZeroPowerAcceleration";
    private static final String LATERAL_ZPA = "lateralZeroPowerAcceleration";
    private static final String RUN_COUNT = "runCount";

    private CalibrationStore() {
    }

    public static void write(Writer writer, CalibrationResult result) throws IOException {
        Properties props = new Properties();
        props.setProperty(X_VELOCITY, Double.toString(result.getXVelocity()));
        props.setProperty(Y_VELOCITY, Double.toString(result.getYVelocity()));
        props.setProperty(FORWARD_ZPA, Double.toString(result.getForwardZeroPowerAcceleration()));
        props.setProperty(LATERAL_ZPA, Double.toString(result.getLateralZeroPowerAcceleration()));
        props.setProperty(RUN_COUNT, Integer.toString(result.getRunCount()));
        props.store(writer, "Pedro Pathing self-calibration results");
    }

    public static CalibrationResult read(Reader reader) throws IOException {
        Properties props = new Properties();
        props.load(reader);
        return new CalibrationResult(
                requireDouble(props, X_VELOCITY),
                requireDouble(props, Y_VELOCITY),
                requireDouble(props, FORWARD_ZPA),
                requireDouble(props, LATERAL_ZPA),
                Integer.parseInt(props.getProperty(RUN_COUNT, "0")));
    }

    private static double requireDouble(Properties props, String key) throws IOException {
        String value = props.getProperty(key);
        if (value == null) {
            throw new IOException("missing calibration property: " + key);
        }
        try {
            return Double.parseDouble(value);
        } catch (NumberFormatException e) {
            throw new IOException("bad number for " + key + ": " + value, e);
        }
    }
}
