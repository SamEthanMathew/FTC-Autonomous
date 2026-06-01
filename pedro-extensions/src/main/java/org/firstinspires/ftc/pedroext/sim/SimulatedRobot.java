package org.firstinspires.ftc.pedroext.sim;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.paths.PathConstraints;

/**
 * Convenience factory that wires the {@link RobotPlant}, {@link SimulatedDrivetrain},
 * {@link SimulatedLocalizer}, and the real Pedro {@link Follower} together into a
 * ready-to-drive simulated robot. Reused by the Phase B tests and the Phase C
 * auto-tuner so both exercise the identical headless setup.
 */
public final class SimulatedRobot {

    /** Pedro mecanum default forward velocity (in/s). */
    public static final double DEFAULT_X_VELOCITY = 81.34;
    /** Pedro mecanum default lateral velocity (in/s). */
    public static final double DEFAULT_Y_VELOCITY = 65.43;
    /** Reasonable mecanum yaw-rate cap (rad/s). */
    public static final double DEFAULT_MAX_ANGULAR_VELOCITY = 4.0;
    /** Heading first-order time constant (s); Pedro has no heading coast constant. */
    public static final double DEFAULT_HEADING_TIME_CONSTANT = 0.12;

    private final FollowerConstants constants;
    private final RobotPlant plant;
    private final SimulatedDrivetrain drivetrain;
    private final SimulatedLocalizer localizer;
    private final Follower follower;
    private final PathConstraints pathConstraints;

    private SimulatedRobot(FollowerConstants constants, RobotPlant plant, SimulatedDrivetrain drivetrain,
                           SimulatedLocalizer localizer, Follower follower, PathConstraints pathConstraints) {
        this.constants = constants;
        this.plant = plant;
        this.drivetrain = drivetrain;
        this.localizer = localizer;
        this.follower = follower;
        this.pathConstraints = pathConstraints;
    }

    public static SimulatedRobot create(FollowerConstants constants) {
        return create(constants, DEFAULT_X_VELOCITY, DEFAULT_Y_VELOCITY, DEFAULT_MAX_ANGULAR_VELOCITY,
                DEFAULT_HEADING_TIME_CONSTANT, 1.0, 0.0, 0.0, 0L);
    }

    /** With sensor noise enabled on the localizer. */
    public static SimulatedRobot createNoisy(FollowerConstants constants, double posNoiseStd,
                                             double headingNoiseStd, long seed) {
        return create(constants, DEFAULT_X_VELOCITY, DEFAULT_Y_VELOCITY, DEFAULT_MAX_ANGULAR_VELOCITY,
                DEFAULT_HEADING_TIME_CONSTANT, 1.0, posNoiseStd, headingNoiseStd, seed);
    }

    public static SimulatedRobot create(FollowerConstants constants, double xVelocity, double yVelocity,
                                        double maxAngularVelocity, double headingTimeConstant,
                                        double maxPowerScaling, double posNoiseStd, double headingNoiseStd,
                                        long seed) {
        RobotPlant plant = RobotPlant.fromConstants(constants, xVelocity, yVelocity, maxAngularVelocity, headingTimeConstant);
        SimulatedDrivetrain drivetrain = new SimulatedDrivetrain(plant, xVelocity, yVelocity, maxAngularVelocity, maxPowerScaling);
        SimulatedLocalizer localizer = (posNoiseStd > 0.0 || headingNoiseStd > 0.0)
                ? new SimulatedLocalizer(plant, posNoiseStd, headingNoiseStd, seed)
                : new SimulatedLocalizer(plant);
        PathConstraints constraints = PathConstraints.defaultConstraints;
        Follower follower = new Follower(constants, localizer, drivetrain, constraints);
        return new SimulatedRobot(constants, plant, drivetrain, localizer, follower, constraints);
    }

    public FollowerConstants getConstants() { return constants; }
    public RobotPlant getPlant() { return plant; }
    public SimulatedDrivetrain getDrivetrain() { return drivetrain; }
    public SimulatedLocalizer getLocalizer() { return localizer; }
    public Follower getFollower() { return follower; }
    public PathConstraints getPathConstraints() { return pathConstraints; }
}
