package org.firstinspires.ftc.pedroext.fusion;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** JVM tests for the vision-fusion EKF against synthetic odometry+vision streams. */
class FusionTest {

    private static final double EPS = 1e-9;

    // --- PoseEKF -------------------------------------------------------------

    @Test
    void predictAddsOdometryDisplacement() {
        PoseEKF ekf = new PoseEKF(0, 0, 0, 1.0);
        ekf.predict(1.0, 2.0, 0.1, new double[]{0.01, 0.01, 0.001});
        assertEquals(1.0, ekf.getX(), EPS);
        assertEquals(2.0, ekf.getY(), EPS);
        assertEquals(0.1, ekf.getHeading(), EPS);
    }

    @Test
    void updatePullsTowardMeasurement() {
        PoseEKF ekf = new PoseEKF(0, 0, 0, 1.0);
        ekf.update(1.0, 0.0, 0.0, new double[]{1.0, 1.0, 1.0}, 10.0, 10.0);
        assertEquals(0.5, ekf.getX(), 1e-6); // K = P/(P+R) = 0.5
    }

    @Test
    void largeCorrectionIsCappedForSmoothness() {
        PoseEKF ekf = new PoseEKF(0, 0, 0, 100.0); // high prior variance => gain ~1
        ekf.update(10.0, 0.0, 0.0, new double[]{1.0, 1.0, 1.0}, 2.0, Math.toRadians(5));
        assertEquals(2.0, ekf.getX(), 1e-6); // uncapped ~9.9, capped to 2.0
    }

    // --- VisionCovariance ----------------------------------------------------

    @Test
    void qualityGateRejectsBadFixes() {
        VisionCovariance cov = VisionCovariance.defaults();
        assertTrue(cov.isReliable(qualityObs(12.0, 0.0, 60.0)));
        assertFalse(cov.isReliable(qualityObs(12.0, 0.0, 5.0)), "low decision margin");
        assertFalse(cov.isReliable(qualityObs(200.0, 0.0, 60.0)), "too far");
        assertFalse(cov.isReliable(qualityObs(12.0, Math.toRadians(70), 60.0)), "too steep");
    }

    @Test
    void covarianceGrowsWithRangeAndAngle() {
        VisionCovariance cov = VisionCovariance.defaults();
        double near = cov.diagonalR(qualityObs(6.0, 0.0, 60.0))[0];
        double far = cov.diagonalR(qualityObs(60.0, 0.0, 60.0))[0];
        double steep = cov.diagonalR(qualityObs(6.0, Math.toRadians(30), 60.0))[0];
        assertTrue(far > near, "farther => less trust (larger variance)");
        assertTrue(steep > near, "steeper => less trust");
    }

    // --- TagMap (season config) ---------------------------------------------

    @Test
    void decodeTagMapExcludesObeliskTags() {
        TagMap map = TagMap.decodeDefault();
        assertTrue(map.isLocalizationTag(20));
        assertTrue(map.isLocalizationTag(24));
        assertFalse(map.isLocalizationTag(21), "Obelisk tag is not for localization");
        assertFalse(map.isLocalizationTag(22));
        assertFalse(map.isLocalizationTag(23));
    }

    // --- Integration: bounded drift + smooth recovery ------------------------

    @Test
    void visionBoundsUnboundedOdometryDrift() {
        StubLocalizer odom = new StubLocalizer();
        odom.set(0, 0, 0);
        VisionFusedLocalizer fused = new VisionFusedLocalizer(odom, TagMap.decodeDefault(), VisionCovariance.defaults());
        fused.setStartPose(new Pose(0, 0, 0));

        double maxFusedError = 0;
        double finalOdomError = 0;
        for (int step = 1; step <= 200; step++) {
            double truthX = 0.5 * step;
            double truthY = 0.0;
            double driftY = 0.02 * step;           // odometry drifts steadily in y
            odom.set(truthX, truthY + driftY, 0);  // odometry = truth + drift

            if (step % 5 == 0) {                   // periodic good tag fix at the truth
                fused.offerVisionObservation(obs(20, truthX, truthY, 0.0, 12.0, 0.0, 60.0));
            }
            fused.update();

            maxFusedError = Math.max(maxFusedError, error(fused.getPose(), truthX, truthY));
            finalOdomError = Math.abs(driftY);
        }
        assertTrue(finalOdomError > 3.5, "odometry alone should diverge, was " + finalOdomError);
        assertTrue(maxFusedError < 1.5, "fused estimate should stay bounded, was " + maxFusedError);
    }

    @Test
    void recoversSmoothlyFromCollisionJump() {
        StubLocalizer odom = new StubLocalizer();
        odom.set(0, 0, 0);
        VisionFusedLocalizer fused = new VisionFusedLocalizer(odom, TagMap.decodeDefault(), VisionCovariance.defaults());
        fused.setStartPose(new Pose(0, 0, 0));

        double collisionShove = 6.0;  // robot shoved +6 in y at step 100; odometry slips (misses it)
        double prevX = 0, prevY = 0, maxStep = 0;
        double errorJustAfter = 0, errorAtEnd = 0;

        for (int step = 1; step <= 240; step++) {
            double truthX = 0.5 * step;
            double truthY = (step >= 100) ? collisionShove : 0.0;
            odom.set(truthX, 0.0, 0);   // odometry tracks x, never sees the shove in y

            if (step % 3 == 0) {
                fused.offerVisionObservation(obs(20, truthX, truthY, 0.0, 12.0, 0.0, 60.0));
            }
            fused.update();

            Pose p = fused.getPose();
            if (step > 1) {
                maxStep = Math.max(maxStep, Math.hypot(p.getX() - prevX, p.getY() - prevY));
            }
            prevX = p.getX();
            prevY = p.getY();
            if (step == 101) {
                errorJustAfter = error(p, truthX, truthY);
            }
            if (step == 240) {
                errorAtEnd = error(p, truthX, truthY);
            }
        }
        assertTrue(errorJustAfter > 3.0, "collision should perturb the estimate, was " + errorJustAfter);
        assertTrue(errorAtEnd < 1.5, "should recover to truth, final error " + errorAtEnd);
        assertTrue(maxStep < 3.0, "no per-loop pose jump from corrections, max step " + maxStep);
    }

    // ---------------------------------------------------------------- helpers

    /** Observation for quality/gate tests (pose irrelevant). */
    private static AprilTagObservation qualityObs(double range, double bearing, double margin) {
        return new AprilTagObservation(20, 0.0, 0.0, 0.0, range, bearing, margin);
    }

    private static AprilTagObservation obs(int id, double x, double y, double heading, double range,
                                           double bearing, double margin) {
        return new AprilTagObservation(id, x, y, heading, range, bearing, margin);
    }

    private static double error(Pose p, double truthX, double truthY) {
        return Math.hypot(p.getX() - truthX, p.getY() - truthY);
    }

    /** A programmable odometry source for the synthetic streams. */
    private static final class StubLocalizer implements Localizer {
        private Pose pose = new Pose(0, 0, 0);

        void set(double x, double y, double heading) {
            this.pose = new Pose(x, y, heading);
        }

        @Override public Pose getPose() { return pose; }
        @Override public Pose getVelocity() { return new Pose(0, 0, 0); }
        @Override public Vector getVelocityVector() { return new Vector(); }
        @Override public void setStartPose(Pose p) { this.pose = p; }
        @Override public void setPose(Pose p) { this.pose = p; }
        @Override public void update() { }
        @Override public double getTotalHeading() { return pose.getHeading(); }
        @Override public double getForwardMultiplier() { return 1.0; }
        @Override public double getLateralMultiplier() { return 1.0; }
        @Override public double getTurningMultiplier() { return 1.0; }
        @Override public void resetIMU() { }
        @Override public double getIMUHeading() { return pose.getHeading(); }
        @Override public boolean isNAN() { return false; }
    }
}
