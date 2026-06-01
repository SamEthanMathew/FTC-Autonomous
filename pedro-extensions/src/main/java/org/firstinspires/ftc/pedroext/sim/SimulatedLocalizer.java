package org.firstinspires.ftc.pedroext.sim;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;

import java.util.Random;

/**
 * A Pedro {@link Localizer} that reports the {@link RobotPlant}'s integrated
 * pose, so the real follower's pose estimate comes from the simulator.
 *
 * <p>Optional Gaussian sensor noise can be enabled to test the follower's
 * correction behavior. The noise offset is resampled on each {@link #update()}
 * and held until the next one, mimicking a per-loop sensor reading (rather than
 * jittering between multiple {@code getPose()} calls within one control loop).
 */
public final class SimulatedLocalizer implements Localizer {

    private final RobotPlant plant;
    private final double positionNoiseStd;   // inches, 1-sigma
    private final double headingNoiseStd;    // radians, 1-sigma
    private final Random random;

    private double noiseX, noiseY, noiseHeading;

    public SimulatedLocalizer(RobotPlant plant) {
        this(plant, 0.0, 0.0, 0L);
    }

    public SimulatedLocalizer(RobotPlant plant, double positionNoiseStd, double headingNoiseStd, long seed) {
        this.plant = plant;
        this.positionNoiseStd = Math.max(0.0, positionNoiseStd);
        this.headingNoiseStd = Math.max(0.0, headingNoiseStd);
        this.random = new Random(seed);
    }

    @Override
    public Pose getPose() {
        return new Pose(plant.getX() + noiseX, plant.getY() + noiseY,
                RobotPlant.normalizeAngle(plant.getHeading() + noiseHeading));
    }

    @Override
    public Pose getVelocity() {
        return new Pose(plant.getFieldVelocityX(), plant.getFieldVelocityY(), plant.getAngularVelocity());
    }

    @Override
    public Vector getVelocityVector() {
        Vector v = new Vector();
        v.setOrthogonalComponents(plant.getFieldVelocityX(), plant.getFieldVelocityY());
        return v;
    }

    @Override
    public void setStartPose(Pose setStart) {
        plant.setPose(setStart.getX(), setStart.getY(), setStart.getHeading());
    }

    @Override
    public void setPose(Pose setPose) {
        plant.setPose(setPose.getX(), setPose.getY(), setPose.getHeading());
    }

    @Override
    public void update() {
        if (positionNoiseStd > 0.0) {
            noiseX = random.nextGaussian() * positionNoiseStd;
            noiseY = random.nextGaussian() * positionNoiseStd;
        }
        if (headingNoiseStd > 0.0) {
            noiseHeading = random.nextGaussian() * headingNoiseStd;
        }
    }

    @Override
    public double getTotalHeading() {
        return plant.getTotalHeading();
    }

    @Override
    public double getForwardMultiplier() {
        return 1.0;
    }

    @Override
    public double getLateralMultiplier() {
        return 1.0;
    }

    @Override
    public double getTurningMultiplier() {
        return 1.0;
    }

    @Override
    public void resetIMU() {
        // No IMU in the simulator.
    }

    @Override
    public double getIMUHeading() {
        return plant.getHeading();
    }

    @Override
    public boolean isNAN() {
        return !plant.isFinite();
    }
}
