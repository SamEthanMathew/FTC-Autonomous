package org.firstinspires.ftc.pedroext.sim;

import java.util.function.Consumer;

/**
 * A disturbance applied to the {@link RobotPlant} once, the first loop at or
 * after a scheduled time — used to test the follower's correction/recovery
 * behavior (a collision-style position kick, a sudden added mass, etc.).
 */
public final class ScheduledDisturbance {

    private final double timeSeconds;
    private final Consumer<RobotPlant> action;
    private boolean applied;

    public ScheduledDisturbance(double timeSeconds, Consumer<RobotPlant> action) {
        this.timeSeconds = timeSeconds;
        this.action = action;
    }

    /** A collision-style instantaneous position/heading kick (field units, radians). */
    public static ScheduledDisturbance positionKick(double timeSeconds, double dx, double dy, double dHeading) {
        return new ScheduledDisturbance(timeSeconds, plant -> plant.applyPositionKick(dx, dy, dHeading));
    }

    /** Suddenly loads extra mass (kg), slowing the response. */
    public static ScheduledDisturbance addMass(double timeSeconds, double kilograms) {
        return new ScheduledDisturbance(timeSeconds, plant -> plant.setAddedMass(kilograms));
    }

    void applyIfDue(double tSeconds, RobotPlant plant) {
        if (!applied && tSeconds >= timeSeconds) {
            action.accept(plant);
            applied = true;
        }
    }

    /** Allows reuse of the same schedule across multiple runs. */
    public void reset() {
        applied = false;
    }
}
