package org.firstinspires.ftc.pedroext.fusion;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;

/**
 * A Pedro {@link Localizer} that wraps an odometry localizer and fuses in AprilTag
 * corrections through a {@link PoseEKF}. Odometry drives the prediction every loop;
 * a vision fix (offered via {@link #offerVisionObservation}) is applied only if its
 * tag is a valid localization tag ({@link TagMap}) and it passes the fix-quality
 * gate ({@link VisionCovariance}), weighted by its covariance and smoothed so the
 * follower never sees a pose discontinuity.
 *
 * <p>Velocity is delegated to the odometry localizer (vision doesn't measure it).
 * The follower sees only the fused pose, so it stays untouched (extension seam, not
 * a fork).
 */
public final class VisionFusedLocalizer implements Localizer {

    private final Localizer odometry;
    private final TagMap tagMap;
    private final VisionCovariance covariance;
    private final double[] processNoise;       // per-update Q diagonal
    private final double maxPositionCorrection; // smoothing caps
    private final double maxHeadingCorrection;
    private final double initialVariance;

    private final PoseEKF ekf;
    private Pose lastOdometryPose;
    private AprilTagObservation pendingObservation;

    public VisionFusedLocalizer(Localizer odometry, TagMap tagMap, VisionCovariance covariance) {
        this(odometry, tagMap, covariance,
                new double[]{0.02, 0.02, 0.0005}, // odometry process noise / loop
                2.0,                                // max 2 in correction / loop
                Math.toRadians(5.0),                // max 5 deg correction / loop
                1.0);
    }

    public VisionFusedLocalizer(Localizer odometry, TagMap tagMap, VisionCovariance covariance,
                                double[] processNoise, double maxPositionCorrection,
                                double maxHeadingCorrection, double initialVariance) {
        this.odometry = odometry;
        this.tagMap = tagMap;
        this.covariance = covariance;
        this.processNoise = processNoise.clone();
        this.maxPositionCorrection = maxPositionCorrection;
        this.maxHeadingCorrection = maxHeadingCorrection;
        this.initialVariance = initialVariance;
        Pose start = odometry.getPose();
        this.ekf = new PoseEKF(start.getX(), start.getY(), start.getHeading(), initialVariance);
        this.lastOdometryPose = start;
    }

    /** Offers the latest AprilTag fix; consumed by the next {@link #update()}. */
    public void offerVisionObservation(AprilTagObservation observation) {
        this.pendingObservation = observation;
    }

    @Override
    public void update() {
        odometry.update();
        Pose current = odometry.getPose();

        // Predict from the odometry displacement since the last loop.
        double dx = current.getX() - lastOdometryPose.getX();
        double dy = current.getY() - lastOdometryPose.getY();
        double dHeading = PoseEKF.normalizeAngle(current.getHeading() - lastOdometryPose.getHeading());
        ekf.predict(dx, dy, dHeading, processNoise);
        lastOdometryPose = current;

        // Correct with a gated, covariance-weighted vision fix, if available.
        AprilTagObservation obs = pendingObservation;
        pendingObservation = null;
        if (obs != null && tagMap.isLocalizationTag(obs.getTagId()) && covariance.isReliable(obs)) {
            ekf.update(obs.getX(), obs.getY(), obs.getHeading(), covariance.diagonalR(obs),
                    maxPositionCorrection, maxHeadingCorrection);
        }
    }

    @Override
    public Pose getPose() {
        return new Pose(ekf.getX(), ekf.getY(), ekf.getHeading());
    }

    @Override
    public Pose getVelocity() {
        return odometry.getVelocity();
    }

    @Override
    public Vector getVelocityVector() {
        return odometry.getVelocityVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        odometry.setStartPose(setStart);
        ekf.setState(setStart.getX(), setStart.getY(), setStart.getHeading(), initialVariance);
        lastOdometryPose = odometry.getPose();
    }

    @Override
    public void setPose(Pose setPose) {
        odometry.setPose(setPose);
        ekf.setState(setPose.getX(), setPose.getY(), setPose.getHeading(), initialVariance);
        lastOdometryPose = odometry.getPose();
    }

    @Override
    public double getTotalHeading() {
        return odometry.getTotalHeading();
    }

    @Override
    public double getForwardMultiplier() {
        return odometry.getForwardMultiplier();
    }

    @Override
    public double getLateralMultiplier() {
        return odometry.getLateralMultiplier();
    }

    @Override
    public double getTurningMultiplier() {
        return odometry.getTurningMultiplier();
    }

    @Override
    public void resetIMU() throws InterruptedException {
        odometry.resetIMU();
    }

    @Override
    public double getIMUHeading() {
        return odometry.getIMUHeading();
    }

    @Override
    public boolean isNAN() {
        return !(Double.isFinite(ekf.getX()) && Double.isFinite(ekf.getY()) && Double.isFinite(ekf.getHeading()));
    }

    /** Current fused-position uncertainty (covariance trace of x,y) — for telemetry. */
    public double getPositionVariance() {
        return ekf.getPositionVariance();
    }
}
