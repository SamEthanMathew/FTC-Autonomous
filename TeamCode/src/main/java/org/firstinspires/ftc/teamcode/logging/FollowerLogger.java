package org.firstinspires.ftc.teamcode.logging;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.pedroext.logging.DataLogWriter;
import org.firstinspires.ftc.pedroext.logging.LogFrame;
import org.firstinspires.ftc.pedroext.voltage.VoltageCompensationModel;

import java.io.IOException;

/**
 * Phase D (on-robot glue) — captures one {@link LogFrame} per control loop from a
 * live {@link Follower} and writes it through a {@link DataLogWriter} (CSV or
 * WPILOG). The frames use the same schema the simulator emits, so a real run can
 * be overlaid on its predicted sim trace in AdvantageScope.
 *
 * <p>HARDWARE-VALIDATION REQUIRED. Reuses the Phase A {@link VoltageCompensationModel}
 * to log commanded vs. applied (voltage-compensated) power. The on-robot file
 * write and the commanded/applied derivation are unverified here.
 */
public final class FollowerLogger {

    private final Follower follower;
    private final VoltageSensor voltageSensor;
    private final VoltageCompensationModel voltageModel;
    private final DataLogWriter writer;
    private final long startNanos;

    public FollowerLogger(Follower follower, VoltageSensor voltageSensor,
                          VoltageCompensationModel voltageModel, DataLogWriter writer) {
        this.follower = follower;
        this.voltageSensor = voltageSensor;
        this.voltageModel = voltageModel;
        this.writer = writer;
        this.startNanos = System.nanoTime();
    }

    /** Call once per loop, after {@code follower.update()}. */
    public void log() throws IOException {
        double t = (System.nanoTime() - startNanos) / 1e9;
        double voltage = voltageSensor.getVoltage();

        Pose pose = follower.getPose();
        Pose target = follower.getClosestPose() != null ? follower.getClosestPose().getPose() : new Pose();

        double commandedPower = follower.getDriveVector().getMagnitude();
        double appliedPower = voltageModel.apply(commandedPower, voltage);

        writer.writeFrame(LogFrame.builder()
                .set("t", t)
                .set("poseX", pose.getX())
                .set("poseY", pose.getY())
                .set("poseHeading", pose.getHeading())
                .set("targetX", target.getX())
                .set("targetY", target.getY())
                .set("targetHeading", target.getHeading())
                .set("trackingError", follower.getTranslationalError().getMagnitude())
                .set("headingError", follower.getHeadingError())
                .set("correctiveX", follower.getCorrectiveVector().getXComponent())
                .set("correctiveY", follower.getCorrectiveVector().getYComponent())
                .set("headingVecX", follower.getHeadingVector().getXComponent())
                .set("headingVecY", follower.getHeadingVector().getYComponent())
                .set("driveX", follower.getDriveVector().getXComponent())
                .set("driveY", follower.getDriveVector().getYComponent())
                .set("centripetalX", follower.getCentripetalForceCorrection().getXComponent())
                .set("centripetalY", follower.getCentripetalForceCorrection().getYComponent())
                .set("commandedPower", commandedPower)
                .set("appliedPower", appliedPower)
                .set("batteryVoltage", voltage)
                .build());
    }

    public void close() throws IOException {
        writer.close();
    }
}
