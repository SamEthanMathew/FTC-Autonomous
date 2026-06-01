package org.firstinspires.ftc.pedroext.logging;

import org.firstinspires.ftc.pedroext.sim.SimulationResult;
import org.firstinspires.ftc.pedroext.sim.TraceSample;

import java.io.IOException;

/**
 * Adapters that emit simulator output through the shared {@link LogFrame} schema,
 * so a predicted (sim) run logs in exactly the same format as an actual (robot)
 * run and the two can be overlaid in AdvantageScope.
 */
public final class LogFrames {

    private LogFrames() {
    }

    /** Converts a simulator {@link TraceSample} to a {@link LogFrame}. */
    public static LogFrame fromTraceSample(TraceSample s) {
        return LogFrame.builder()
                .set("t", s.t)
                .set("poseX", s.actualX)
                .set("poseY", s.actualY)
                .set("poseHeading", s.actualHeading)
                .set("targetX", s.targetX)
                .set("targetY", s.targetY)
                .set("targetHeading", s.targetHeading)
                .set("trackingError", s.trackingError)
                .set("headingError", s.headingError)
                .set("correctiveX", s.correctiveX)
                .set("correctiveY", s.correctiveY)
                .set("headingVecX", s.headingVecX)
                .set("headingVecY", s.headingVecY)
                .set("driveX", s.driveX)
                .set("driveY", s.driveY)
                .set("centripetalX", s.centripetalX)
                .set("centripetalY", s.centripetalY)
                // The basic sim has no per-wheel power/voltage-compensation stage, so
                // commanded == applied; battery voltage is the (constant) sim voltage.
                .set("commandedPower", 0.0)
                .set("appliedPower", 0.0)
                .set("batteryVoltage", s.voltage)
                .build();
    }

    /** Writes every sample of a simulated run to the given log writer. */
    public static void write(SimulationResult result, DataLogWriter writer) throws IOException {
        for (TraceSample s : result.getSamples()) {
            writer.writeFrame(fromTraceSample(s));
        }
    }
}
