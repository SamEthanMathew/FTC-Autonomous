package org.firstinspires.ftc.pedroext.fusion;

/**
 * Turns an {@link AprilTagObservation}'s quality into (a) a yes/no reliability gate
 * and (b) a measurement covariance for the EKF. Trust is highest for a near,
 * centered, high-margin detection and degrades with range and off-center angle —
 * so a far/steep tag nudges the estimate gently while a close/centered one
 * corrects firmly.
 */
public final class VisionCovariance {

    private final double basePositionStd;   // inches at zero range, centered
    private final double baseHeadingStd;     // radians, "
    private final double rangeFactorPerInch; // variance growth with range
    private final double bearingFactorPerRad;// variance growth with off-center angle
    private final double minDecisionMargin;  // gate: reject below this
    private final double maxRangeInches;     // gate: reject beyond this
    private final double maxBearingRadians;  // gate: reject steeper than this

    public VisionCovariance(double basePositionStd, double baseHeadingStd, double rangeFactorPerInch,
                            double bearingFactorPerRad, double minDecisionMargin, double maxRangeInches,
                            double maxBearingRadians) {
        this.basePositionStd = basePositionStd;
        this.baseHeadingStd = baseHeadingStd;
        this.rangeFactorPerInch = rangeFactorPerInch;
        this.bearingFactorPerRad = bearingFactorPerRad;
        this.minDecisionMargin = minDecisionMargin;
        this.maxRangeInches = maxRangeInches;
        this.maxBearingRadians = maxBearingRadians;
    }

    /** Sensible FTC defaults; tune per camera. */
    public static VisionCovariance defaults() {
        return new VisionCovariance(
                /* basePositionStd   */ 1.0,    // ~1 in at close range
                /* baseHeadingStd    */ Math.toRadians(2.0),
                /* rangeFactorPerInch*/ 0.05,
                /* bearingFactorPerRad*/ 1.5,
                /* minDecisionMargin */ 20.0,
                /* maxRangeInches    */ 72.0,
                /* maxBearingRadians */ Math.toRadians(45.0));
    }

    /** Whether this detection passes the fix-quality gate (margin/range/angle). */
    public boolean isReliable(AprilTagObservation o) {
        return o.getDecisionMargin() >= minDecisionMargin
                && o.getRangeInches() <= maxRangeInches
                && Math.abs(o.getBearingRadians()) <= maxBearingRadians;
    }

    /** Diagonal measurement covariance {@code [varX, varY, varHeading]} for the EKF. */
    public double[] diagonalR(AprilTagObservation o) {
        double scale = (1.0 + rangeFactorPerInch * o.getRangeInches())
                * (1.0 + bearingFactorPerRad * Math.abs(o.getBearingRadians()));
        double posStd = basePositionStd * scale;
        double headingStd = baseHeadingStd * scale;
        return new double[]{posStd * posStd, posStd * posStd, headingStd * headingStd};
    }
}
