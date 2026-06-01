package org.firstinspires.ftc.teamcode.logging;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.pedroext.logging.WpilogWriter;
import org.firstinspires.ftc.pedroext.voltage.VoltageCompensationModel;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.io.File;
import java.io.FileOutputStream;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Phase D demo — follows a short path while logging every loop to a timestamped
 * WPILOG file (loadable in AdvantageScope; overlay it on the matching sim trace).
 *
 * <p>HARDWARE-VALIDATION REQUIRED. File writes and on-robot behavior are unverified.
 */
@Autonomous(name = "Logging Demo (Pedro)", group = "logging")
public class LoggingDemoOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws Exception {
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        String name = "pedro_log_" + new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date()) + ".wpilog";
        File file = new File(AppUtil.FIRST_FOLDER, name);

        Path path = new Path(new BezierLine(new Pose(0, 0, 0), new Pose(48, 0, 0)));
        path.setConstantHeadingInterpolation(0);

        telemetry.addLine("Press START to follow + log to " + name);
        telemetry.update();
        waitForStart();

        try (FileOutputStream out = new FileOutputStream(file)) {
            WpilogWriter writer = new WpilogWriter(out);
            FollowerLogger logger = new FollowerLogger(follower, voltageSensor,
                    VoltageCompensationModel.pedroDefaults(), writer);

            follower.followPath(path, true);
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                logger.log();
            }
            logger.close();
        }

        telemetry.addData("saved", file.getAbsolutePath());
        telemetry.update();
        while (opModeIsActive()) {
            sleep(100);
        }
    }
}
