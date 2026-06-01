package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.pedroext.fusion.AprilTagObservation;
import org.firstinspires.ftc.pedroext.fusion.TagMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * Phase E (on-robot glue) — turns AprilTag detections into {@link AprilTagObservation}s
 * for {@link org.firstinspires.ftc.pedroext.fusion.VisionFusedLocalizer}.
 *
 * <p>HARDWARE-VALIDATION REQUIRED. Uses the SDK's built-in {@code detection.robotPose}
 * (requires the current-season tag library with known field positions and a
 * configured camera pose) to get the robot's field pose; range/bearing/decisionMargin
 * drive the fusion's quality gate + covariance. Only tags in the {@link TagMap} are
 * returned. The camera calibration, camera-on-robot pose, and tag library must be
 * set up and verified on the robot — none of that is exercised here.
 */
public final class AprilTagVisionSource {

    private final AprilTagProcessor processor;
    private final VisionPortal portal;
    private final TagMap tagMap;

    public AprilTagVisionSource(HardwareMap hardwareMap, String webcamName, TagMap tagMap) {
        this.tagMap = tagMap;
        this.processor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .build();
        this.portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                .addProcessor(processor)
                .build();
    }

    /**
     * Returns the best current localization-tag observation (closest reliable tag),
     * or {@code null} if none. Feed the result to
     * {@code VisionFusedLocalizer.offerVisionObservation(...)} each loop.
     */
    public AprilTagObservation poll() {
        AprilTagObservation best = null;
        double bestRange = Double.MAX_VALUE;
        for (AprilTagDetection detection : processor.getDetections()) {
            if (detection.metadata == null || detection.robotPose == null) {
                continue;
            }
            if (!tagMap.isLocalizationTag(detection.id)) {
                continue; // skip non-localization tags (e.g. DECODE Obelisk 21-23)
            }
            if (detection.ftcPose.range < bestRange) {
                bestRange = detection.ftcPose.range;
                best = toObservation(detection);
            }
        }
        return best;
    }

    private static AprilTagObservation toObservation(AprilTagDetection d) {
        double x = d.robotPose.getPosition().x;
        double y = d.robotPose.getPosition().y;
        double heading = d.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);
        return new AprilTagObservation(d.id, x, y, heading,
                d.ftcPose.range, d.ftcPose.bearing, d.decisionMargin);
    }

    public void close() {
        portal.close();
    }
}
