package org.firstinspires.ftc.pedroext.fusion;

/**
 * One AprilTag-derived absolute pose fix: the robot pose (field frame) implied by
 * the detection, plus the quality signals used to weight/gate it (range, how far
 * off-center the tag was, and the detector's decision margin).
 *
 * <p>The pose is computed on the robot from the tag's field pose and the
 * VisionPortal's tag-relative measurement; this pure-Java type carries the result
 * so the fusion logic is testable without Android.
 */
public final class AprilTagObservation {

    private final int tagId;
    private final double x;
    private final double y;
    private final double heading;
    private final double rangeInches;
    private final double bearingRadians;
    private final double decisionMargin;

    public AprilTagObservation(int tagId, double x, double y, double heading,
                               double rangeInches, double bearingRadians, double decisionMargin) {
        this.tagId = tagId;
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.rangeInches = rangeInches;
        this.bearingRadians = bearingRadians;
        this.decisionMargin = decisionMargin;
    }

    public int getTagId() { return tagId; }
    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }
    public double getRangeInches() { return rangeInches; }
    public double getBearingRadians() { return bearingRadians; }
    public double getDecisionMargin() { return decisionMargin; }
}
