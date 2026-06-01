package org.firstinspires.ftc.teamcode.localization;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.pedroext.fusion.AprilTagObservation;
import org.firstinspires.ftc.pedroext.fusion.TagMap;
import org.firstinspires.ftc.pedroext.fusion.VisionCovariance;
import org.firstinspires.ftc.pedroext.fusion.VisionFusedLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Phase E demo — drives in TeleOp while localizing with a {@link VisionFusedLocalizer}
 * that wraps the Pinpoint odometry and folds in AprilTag fixes. The follower is
 * untouched: it just receives a different {@code Localizer}.
 *
 * <p>HARDWARE-VALIDATION REQUIRED. Verify the camera config, tag library, camera
 * pose, and that the fused pose stays stable (no jumps) as tags come and go.
 */
@TeleOp(name = "Vision-Fused Localization (Pedro)", group = "localization")
public class VisionFusedLocalizationOpMode extends LinearOpMode {

    public static String WEBCAM_NAME = "Webcam 1";

    @Override
    public void runOpMode() {
        TagMap tagMap = TagMap.decodeDefault();

        // Wrap the odometry localizer with the fusing localizer, then hand THAT to
        // the follower via the standard builder seam (no fork).
        PinpointLocalizer odometry = new PinpointLocalizer(hardwareMap, Constants.localizerConstants);
        VisionFusedLocalizer fused = new VisionFusedLocalizer(odometry, tagMap, VisionCovariance.defaults());

        Follower follower = new FollowerBuilder(Constants.followerConstants, hardwareMap)
                .mecanumDrivetrain(Constants.driveConstants)
                .setLocalizer(fused)
                .pathConstraints(Constants.pathConstraints)
                .build();
        follower.setStartingPose(new Pose(0, 0, 0));

        AprilTagVisionSource vision = new AprilTagVisionSource(hardwareMap, WEBCAM_NAME, tagMap);

        telemetry.addLine("Drive with the sticks; fused pose updates from odometry + AprilTags.");
        telemetry.update();
        waitForStart();

        follower.startTeleopDrive();
        while (opModeIsActive()) {
            AprilTagObservation observation = vision.poll();
            if (observation != null) {
                fused.offerVisionObservation(observation);
            }
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();

            Pose pose = follower.getPose();
            telemetry.addData("fused pose", "x=%.1f y=%.1f h=%.1f°", pose.getX(), pose.getY(),
                    Math.toDegrees(pose.getHeading()));
            telemetry.addData("position variance", "%.3f", fused.getPositionVariance());
            telemetry.addData("tag fix", observation != null ? ("id " + observation.getTagId()) : "none");
            telemetry.update();
        }
        vision.close();
    }
}
