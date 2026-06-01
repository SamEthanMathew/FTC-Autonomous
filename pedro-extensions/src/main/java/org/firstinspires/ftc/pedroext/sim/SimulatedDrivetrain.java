package org.firstinspires.ftc.pedroext.sim;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.math.Vector;

/**
 * A Pedro {@link Drivetrain} backed by the offline {@link RobotPlant} instead of
 * real motors, so the <b>unmodified</b> Pedro follower can run headlessly.
 *
 * <p>The follower drives a {@code Drivetrain} via the base-class method
 * {@code runDrive(corrective, heading, pathing, robotHeading, velocity)}, which
 * calls {@code runDrive(calculateDrive(corrective, heading, pathing, robotHeading))}.
 * We therefore only implement {@link #calculateDrive} (which decodes the three
 * field-frame command vectors into a robot-frame velocity command) and
 * {@link #runDrive(double[])} (which hands that command to the plant).
 *
 * <h2>Command decoding (mirrors Mecanum's effective motion)</h2>
 * Pedro's {@code Mecanum} forms left/right side vectors {@code corrective ∓ heading}
 * and adds {@code pathing}; the <em>average</em> of the two sides is
 * {@code corrective + pathing} (heading cancels) → net translation, and the
 * <em>difference</em> is {@code 2·heading} → yaw. So:
 * <ul>
 *   <li>Net field translation power = {@code corrective + pathing}, magnitude
 *       clamped to {@code maxPowerScaling}; rotated into the robot frame and
 *       scaled by {@code (xVelocity, yVelocity)} to get steady-state forward/
 *       strafe velocity.</li>
 *   <li>Signed yaw power = projection of {@code headingPower} onto the unit
 *       heading vector (Pedro builds {@code headingVector = new Vector(signedPID,
 *       robotHeading)}; positive = CCW). Scaled by {@code maxAngularVelocity}.</li>
 * </ul>
 * The hard-saturation trade-off Pedro makes when heading + translation exceed
 * full power is intentionally not modeled (see {@link RobotPlant} scope notes).
 */
public final class SimulatedDrivetrain extends Drivetrain {

    private final RobotPlant plant;
    private double xVelocity;
    private double yVelocity;
    private final double maxAngularVelocity;
    private double simVoltage = 12.0;

    public SimulatedDrivetrain(RobotPlant plant, double xVelocity, double yVelocity,
                               double maxAngularVelocity, double maxPowerScaling) {
        this.plant = plant;
        this.xVelocity = xVelocity;
        this.yVelocity = yVelocity;
        this.maxAngularVelocity = maxAngularVelocity;
        this.maxPowerScaling = maxPowerScaling;
        this.nominalVoltage = 12.0;
        this.voltageCompensation = false;
    }

    @Override
    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        // Net field-frame translation; clamp magnitude to available power.
        Vector translation = correctivePower.plus(pathingPower);
        if (translation.getMagnitude() > maxPowerScaling) {
            translation.setMagnitude(maxPowerScaling);
        }
        double tx = translation.getXComponent();
        double ty = translation.getYComponent();

        double cos = Math.cos(robotHeading);
        double sin = Math.sin(robotHeading);
        // Field -> robot frame (rotate by -heading).
        double forwardPower = tx * cos + ty * sin;
        double strafePower = -tx * sin + ty * cos;

        // Signed yaw = projection of heading vector onto the unit heading direction.
        double yawPower = headingPower.getXComponent() * cos + headingPower.getYComponent() * sin;

        double forwardVel = forwardPower * xVelocity;
        double strafeVel = strafePower * yVelocity;
        double yawVel = yawPower * maxAngularVelocity;
        return new double[]{forwardVel, strafeVel, yawVel};
    }

    /** Receives {@code {forwardVel, strafeVel, yawVel}} from {@link #calculateDrive}. */
    @Override
    public void runDrive(double[] command) {
        plant.setCommand(command[0], command[1], command[2]);
    }

    @Override
    public void updateConstants() {
        // Nothing to refresh; velocities are set via setXVelocity/setYVelocity.
    }

    @Override
    public void breakFollowing() {
        plant.setCommand(0.0, 0.0, 0.0);
    }

    @Override
    public void startTeleopDrive() { /* no-op in sim */ }

    @Override
    public void startTeleopDrive(boolean brakeMode) { /* no-op in sim */ }

    @Override
    public double xVelocity() { return xVelocity; }

    @Override
    public double yVelocity() { return yVelocity; }

    @Override
    public void setXVelocity(double xMovement) { this.xVelocity = xMovement; }

    @Override
    public void setYVelocity(double yMovement) { this.yVelocity = yMovement; }

    @Override
    public double getVoltage() { return simVoltage; }

    /** Sets the simulated bus voltage (for voltage-compensation / logging tests). */
    public void setSimVoltage(double voltage) { this.simVoltage = voltage; }

    @Override
    public String debugString() {
        return "SimulatedDrivetrain{xVel=" + xVelocity + ", yVel=" + yVelocity
                + ", maxAngVel=" + maxAngularVelocity + ", V=" + simVoltage + "}";
    }
}
