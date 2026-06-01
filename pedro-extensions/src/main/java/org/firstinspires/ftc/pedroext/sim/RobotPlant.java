package org.firstinspires.ftc.pedroext.sim;

import com.pedropathing.follower.FollowerConstants;

/**
 * Discrete-time, first-order dynamics model of a mecanum FTC robot, in the
 * field/robot frames Pedro uses. It consumes a steady-state velocity command
 * (forward, strafe, yaw, in robot frame) and integrates pose.
 *
 * <h2>Model</h2>
 * Each translational velocity component is <b>slew-rate limited</b> toward its
 * commanded steady state: it accelerates at up to {@code maxAcceleration} and
 * decelerates at up to the team's <b>measured zero-power (coast) acceleration</b>.
 * Using the measured zero-power acceleration as a <em>constant</em> deceleration
 * rate (rather than an exponential, viscous relaxation) is deliberate: it matches
 * how Pedro itself models braking — its predictive braking integrates a constant
 * deceleration — so the follower's braking lines up with the plant and the robot
 * stops on the endpoint instead of overshooting. (An exponential first-order
 * model makes deceleration proportional to speed, which is inconsistent with the
 * roughly constant coast deceleration Pedro characterizes.) Heading uses a
 * first-order time constant (Pedro exposes no heading coast-accel constant).
 * Added mass scales the effective accelerations by {@code mass/(mass+addedMass)},
 * modeling a heavier robot's slower response (F = ma).
 *
 * <h2>Scope / divergence from reality (documented honestly)</h2>
 * This is a first-order velocity approximation, not a full dynamics simulation.
 * It deliberately does NOT model: wheel slip, motor back-EMF/torque curves,
 * uneven weight distribution, carpet/tile friction anisotropy beyond the two
 * axes, the saturation trade-off Pedro makes between heading and translation
 * power when both are maxed, or battery sag (handled separately by the voltage
 * model). It is sufficient to validate the closed-loop follower (Phase C) and to
 * compare predicted vs. actual traces (Phase D), and will diverge from a real
 * robot most in hard-saturation, high-slip, and sharp-acceleration regimes.
 *
 * <p>Not thread-safe; drive it from a single simulation loop.
 */
public final class RobotPlant {

    /** Default acceleration as a multiple of the measured coast deceleration. */
    public static final double DEFAULT_ACCEL_TO_DECEL_RATIO = 2.0;

    private final double xVelocity;                 // in/s, max forward speed
    private final double yVelocity;                 // in/s, max strafe speed
    private final double forwardDecel;              // in/s^2, magnitude (> 0)
    private final double lateralDecel;              // in/s^2, magnitude (> 0)
    private final double forwardAccel;              // in/s^2, magnitude (> 0)
    private final double lateralAccel;              // in/s^2, magnitude (> 0)
    private final double maxAngularVelocity;        // rad/s
    private final double headingTimeConstant;       // s
    private final double mass;                       // kg (nominal)

    // Field-frame pose.
    private double x, y, heading;
    private double totalHeading;
    // Robot-frame velocities.
    private double vForward, vStrafe, omega;
    // Commanded steady-state robot-frame velocities.
    private double cmdForward, cmdStrafe, cmdOmega;
    // Disturbance: extra mass currently loaded.
    private double addedMass;

    public RobotPlant(double xVelocity, double yVelocity, double forwardZeroPowerAcceleration,
                      double lateralZeroPowerAcceleration, double forwardAcceleration, double lateralAcceleration,
                      double maxAngularVelocity, double headingTimeConstant, double mass) {
        if (!(xVelocity > 0) || !(yVelocity > 0)) {
            throw new IllegalArgumentException("xVelocity and yVelocity must be > 0");
        }
        if (forwardZeroPowerAcceleration == 0 || lateralZeroPowerAcceleration == 0) {
            throw new IllegalArgumentException("zero-power accelerations must be non-zero");
        }
        if (!(forwardAcceleration > 0) || !(lateralAcceleration > 0)) {
            throw new IllegalArgumentException("accelerations must be > 0");
        }
        if (!(maxAngularVelocity > 0) || !(headingTimeConstant > 0) || !(mass > 0)) {
            throw new IllegalArgumentException("maxAngularVelocity, headingTimeConstant, mass must be > 0");
        }
        this.xVelocity = xVelocity;
        this.yVelocity = yVelocity;
        this.forwardDecel = Math.abs(forwardZeroPowerAcceleration);
        this.lateralDecel = Math.abs(lateralZeroPowerAcceleration);
        this.forwardAccel = Math.abs(forwardAcceleration);
        this.lateralAccel = Math.abs(lateralAcceleration);
        this.maxAngularVelocity = maxAngularVelocity;
        this.headingTimeConstant = headingTimeConstant;
        this.mass = mass;
    }

    /**
     * Builds a plant from the team's {@link FollowerConstants} (which hold the
     * characterized zero-power accelerations and mass) plus the drivetrain's
     * measured max velocities and a heading model. Acceleration defaults to
     * {@link #DEFAULT_ACCEL_TO_DECEL_RATIO}× the measured coast deceleration
     * (motors drive harder than coast friction stops).
     */
    public static RobotPlant fromConstants(FollowerConstants constants, double xVelocity, double yVelocity,
                                           double maxAngularVelocity, double headingTimeConstant) {
        double fwdDecel = Math.abs(constants.forwardZeroPowerAcceleration);
        double latDecel = Math.abs(constants.lateralZeroPowerAcceleration);
        return new RobotPlant(xVelocity, yVelocity,
                constants.forwardZeroPowerAcceleration, constants.lateralZeroPowerAcceleration,
                DEFAULT_ACCEL_TO_DECEL_RATIO * fwdDecel, DEFAULT_ACCEL_TO_DECEL_RATIO * latDecel,
                maxAngularVelocity, headingTimeConstant, constants.mass);
    }

    /** Sets the commanded steady-state robot-frame velocities (in/s, in/s, rad/s). */
    public void setCommand(double forwardVel, double strafeVel, double yawVel) {
        this.cmdForward = forwardVel;
        this.cmdStrafe = strafeVel;
        this.cmdOmega = yawVel;
    }

    /** Advances the simulation by {@code dt} seconds using the current command. */
    public void step(double dt) {
        if (!(dt > 0)) {
            return;
        }
        double massFactor = (mass + addedMass) / mass; // >= 1; heavier => slower
        double tauHeading = headingTimeConstant * massFactor;

        vForward = slew(vForward, cmdForward, dt, forwardAccel / massFactor, forwardDecel / massFactor);
        vStrafe = slew(vStrafe, cmdStrafe, dt, lateralAccel / massFactor, lateralDecel / massFactor);
        omega = relax(omega, cmdOmega, dt, tauHeading);

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double vxField = vForward * cos - vStrafe * sin;
        double vyField = vForward * sin + vStrafe * cos;

        x += vxField * dt;
        y += vyField * dt;
        double dHeading = omega * dt;
        heading = normalizeAngle(heading + dHeading);
        totalHeading += dHeading;
    }

    private static double relax(double current, double target, double dt, double tau) {
        double alpha = 1.0 - Math.exp(-dt / tau);
        return current + (target - current) * alpha;
    }

    /**
     * Slews a velocity toward its target, capping the change rate at
     * {@code accelRate} when speeding up and {@code decelRate} when slowing down
     * (the constant coast/brake deceleration).
     */
    static double slew(double current, double target, double dt, double accelRate, double decelRate) {
        boolean speedingUp = Math.abs(target) >= Math.abs(current)
                && (current == 0.0 || Math.signum(target) == Math.signum(current));
        double rate = speedingUp ? accelRate : decelRate;
        double maxStep = rate * dt;
        double delta = target - current;
        if (Math.abs(delta) <= maxStep) {
            return target;
        }
        return current + Math.copySign(maxStep, delta);
    }

    // --- Disturbances ---------------------------------------------------------

    /** Instantaneous position/heading kick (e.g., a collision), in field units. */
    public void applyPositionKick(double dx, double dy, double dHeading) {
        x += dx;
        y += dy;
        heading = normalizeAngle(heading + dHeading);
        totalHeading += dHeading;
    }

    /** Sets extra loaded mass (kg) — slows the response proportionally. */
    public void setAddedMass(double addedMass) {
        this.addedMass = Math.max(0.0, addedMass);
    }

    // --- State access ---------------------------------------------------------

    public void setPose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = normalizeAngle(heading);
        this.vForward = 0;
        this.vStrafe = 0;
        this.omega = 0;
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }
    public double getTotalHeading() { return totalHeading; }

    /** Field-frame x velocity (in/s). */
    public double getFieldVelocityX() {
        return vForward * Math.cos(heading) - vStrafe * Math.sin(heading);
    }

    /** Field-frame y velocity (in/s). */
    public double getFieldVelocityY() {
        return vForward * Math.sin(heading) + vStrafe * Math.cos(heading);
    }

    public double getAngularVelocity() { return omega; }

    public boolean isFinite() {
        return Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(heading);
    }

    static double normalizeAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }
}
