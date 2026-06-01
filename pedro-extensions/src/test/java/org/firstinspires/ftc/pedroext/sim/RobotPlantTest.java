package org.firstinspires.ftc.pedroext.sim;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Pure-logic tests for the plant dynamics (no follower, deterministic, fast). */
class RobotPlantTest {

    private static final double EPS = 1e-9;

    private RobotPlant plant() {
        // xVel=60, yVel=60, fwd/lat coast decel = 40 in/s^2, accel = 80 in/s^2.
        return new RobotPlant(60, 60, -40, -40, 80, 80, 4.0, 0.1, 10.0);
    }

    @Test
    void acceleratesAtAccelerationRateFromRest() {
        RobotPlant p = plant();
        p.setPose(0, 0, 0);
        p.setCommand(60, 0, 0);
        p.step(0.01);                 // 80 in/s^2 * 0.01 s = 0.8 in/s
        assertEquals(0.8, p.getFieldVelocityX(), 1e-6);
    }

    @Test
    void coastDeceleratesAtMeasuredZeroPowerRate() {
        RobotPlant p = plant();
        p.setPose(0, 0, 0);
        p.setCommand(60, 0, 0);
        for (int i = 0; i < 200; i++) {
            p.step(0.01);             // reach terminal velocity (60 in/s)
        }
        assertEquals(60.0, p.getFieldVelocityX(), 1e-6);
        p.setCommand(0, 0, 0);        // cut power -> coast
        double before = p.getFieldVelocityX();
        p.step(0.01);
        double decel = (before - p.getFieldVelocityX()) / 0.01;
        assertEquals(40.0, decel, 1e-6); // exactly the measured coast deceleration
    }

    @Test
    void reachesCommandedSteadyStateVelocity() {
        RobotPlant p = plant();
        p.setPose(0, 0, 0);
        p.setCommand(30, 0, 0);
        for (int i = 0; i < 300; i++) {
            p.step(0.01);
        }
        assertEquals(30.0, p.getFieldVelocityX(), 1e-6);
        assertTrue(p.getX() > 0, "should have travelled forward");
    }

    @Test
    void velocityIsAppliedAlongHeading() {
        RobotPlant p = plant();
        p.setPose(0, 0, Math.PI / 2.0);   // facing +y
        p.setCommand(30, 0, 0);            // forward command
        for (int i = 0; i < 100; i++) {
            p.step(0.01);
        }
        assertTrue(p.getY() > 1.0, "forward at heading +90 should move +y");
        assertEquals(0.0, p.getX(), 1e-3);
    }

    @Test
    void addedMassSlowsAcceleration() {
        RobotPlant light = plant();
        RobotPlant heavy = plant();
        light.setPose(0, 0, 0);
        heavy.setPose(0, 0, 0);
        heavy.setAddedMass(10.0);          // double the mass
        light.setCommand(60, 0, 0);
        heavy.setCommand(60, 0, 0);
        for (int i = 0; i < 10; i++) {
            light.step(0.01);
            heavy.step(0.01);
        }
        assertTrue(heavy.getFieldVelocityX() < light.getFieldVelocityX(),
                "heavier robot should accelerate slower");
    }

    @Test
    void positionKickOffsetsPose() {
        RobotPlant p = plant();
        p.setPose(10, 10, 0);
        p.applyPositionKick(3, -4, 0);
        assertEquals(13.0, p.getX(), EPS);
        assertEquals(6.0, p.getY(), EPS);
    }

    @Test
    void slewRespectsAccelAndDecelCaps() {
        // speeding up uses accel cap
        assertEquals(0.8, RobotPlant.slew(0.0, 60.0, 0.01, 80.0, 40.0), EPS);
        // slowing down uses decel cap
        assertEquals(59.6, RobotPlant.slew(60.0, 0.0, 0.01, 80.0, 40.0), EPS);
        // small remaining delta snaps to target
        assertEquals(30.0, RobotPlant.slew(29.99, 30.0, 0.01, 80.0, 40.0), EPS);
    }

    @Test
    void normalizeAngleWrapsToPlusMinusPi() {
        assertEquals(0.0, RobotPlant.normalizeAngle(2 * Math.PI), 1e-9);
        assertEquals(Math.PI / 2, RobotPlant.normalizeAngle(Math.PI / 2 + 2 * Math.PI), 1e-9);
        assertTrue(Math.abs(RobotPlant.normalizeAngle(3 * Math.PI)) - Math.PI < 1e-9);
    }

    @Test
    void nonPositiveDtIsNoOp() {
        RobotPlant p = plant();
        p.setPose(5, 5, 0);
        p.setCommand(60, 0, 0);
        p.step(0.0);
        p.step(-1.0);
        assertEquals(5.0, p.getX(), EPS);
        assertEquals(0.0, p.getFieldVelocityX(), EPS);
    }
}
