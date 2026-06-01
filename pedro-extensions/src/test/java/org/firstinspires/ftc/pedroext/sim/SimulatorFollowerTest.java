package org.firstinspires.ftc.pedroext.sim;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.Collections;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Runs the real, unmodified Pedro {@link Follower} against the offline simulator
 * on representative paths and asserts convergence, a bounded error envelope, and
 * disturbance recovery. These tests pace in real time (see {@link SimulationLoop}),
 * so paths are kept short.
 */
class SimulatorFollowerTest {

    // A bit faster than a robot loop, still a sane PIDF dt; short safety timeout.
    private SimulationLoop loop() {
        return new SimulationLoop(0.004, 16.0, 30, 0.05);
    }

    private SimulatedRobot robot() {
        return SimulatedRobot.create(new FollowerConstants());
    }

    private Path line(Pose start, Pose end) {
        Path path = new Path(new BezierLine(start, end));
        path.setConstantHeadingInterpolation(start.getHeading());
        return path;
    }

    @Test
    void straightLineConvergesWithinTolerance() {
        SimulatedRobot r = robot();
        Pose start = new Pose(0, 0, 0);
        Pose end = new Pose(24, 0, 0);
        SimulationResult result = loop().followPath(r.getFollower(), r.getPlant(), r.getLocalizer(),
                r.getDrivetrain(), line(start, end), start);

        assertFalse(result.isTimedOut(), "follow should complete, not time out");
        assertTrue(result.finalPositionErrorTo(end) < 1.0,
                "final position error " + result.finalPositionErrorTo(end) + " in should be < 1.0");
        assertTrue(result.getMaxTrackingError() < 6.0,
                "max tracking error " + result.getMaxTrackingError() + " in should stay within envelope");
    }

    @Test
    void pureStrafeConvergesWithinTolerance() {
        SimulatedRobot r = robot();
        Pose start = new Pose(0, 0, 0);
        Pose end = new Pose(0, 20, 0);  // move +y while facing +x: exercises yVelocity axis
        SimulationResult result = loop().followPath(r.getFollower(), r.getPlant(), r.getLocalizer(),
                r.getDrivetrain(), line(start, end), start);

        assertFalse(result.isTimedOut());
        assertTrue(result.finalPositionErrorTo(end) < 1.0,
                "final strafe error " + result.finalPositionErrorTo(end) + " in should be < 1.0");
    }

    @Test
    void curvedPathWithHeadingChangeConverges() {
        SimulatedRobot r = robot();
        Pose start = new Pose(0, 0, 0);
        Pose end = new Pose(24, 24, Math.PI / 2.0);
        Path curve = new Path(new BezierCurve(Arrays.asList(new Pose(0, 0), new Pose(24, 0), new Pose(24, 24))));
        curve.setLinearHeadingInterpolation(0, Math.PI / 2.0);

        SimulationResult result = loop().followPath(r.getFollower(), r.getPlant(), r.getLocalizer(),
                r.getDrivetrain(), curve, start);

        assertFalse(result.isTimedOut());
        assertTrue(result.finalPositionErrorTo(end) < 2.0,
                "final curve position error " + result.finalPositionErrorTo(end) + " in should be < 2.0");
        assertTrue(result.finalHeadingErrorTo(end) < 0.15,
                "final heading error " + result.finalHeadingErrorTo(end) + " rad should be < 0.15");
    }

    @Test
    void multiSegmentChainReachesFinalEndpoint() {
        SimulatedRobot r = robot();
        Follower follower = r.getFollower();
        Pose start = new Pose(0, 0, 0);
        Pose end = new Pose(24, 24, 0);

        Path seg1 = line(new Pose(0, 0, 0), new Pose(24, 0, 0));
        Path seg2 = line(new Pose(24, 0, 0), new Pose(24, 24, 0));
        PathChain chain = follower.pathBuilder().addPath(seg1).addPath(seg2).build();

        SimulationResult result = loop().followPathChain(follower, r.getPlant(), r.getLocalizer(),
                r.getDrivetrain(), chain, start);

        assertFalse(result.isTimedOut());
        assertTrue(result.finalPositionErrorTo(end) < 2.0,
                "final chain position error " + result.finalPositionErrorTo(end) + " in should be < 2.0");
    }

    @Test
    void recoversFromMidPathPositionKick() {
        SimulatedRobot r = robot();
        Pose start = new Pose(0, 0, 0);
        Pose end = new Pose(36, 0, 0);

        ScheduledDisturbance kick = ScheduledDisturbance.positionKick(1.0, 0.0, 8.0, 0.0); // shove 8" sideways
        SimulationResult result = loop().followPath(r.getFollower(), r.getPlant(), r.getLocalizer(),
                r.getDrivetrain(), line(start, end), start, Collections.singletonList(kick));

        assertFalse(result.isTimedOut());
        // The kick must actually perturb tracking...
        assertTrue(result.maxTrackingErrorAfter(1.0) > 4.0,
                "kick should spike tracking error, saw " + result.maxTrackingErrorAfter(1.0));
        // ...and the follower must recover to the endpoint.
        assertTrue(result.finalPositionErrorTo(end) < 1.5,
                "should recover to endpoint, final error " + result.finalPositionErrorTo(end) + " in");
    }

    @Test
    void producesInspectableTraceDump() throws IOException {
        SimulatedRobot r = robot();
        Pose start = new Pose(0, 0, 0);
        Pose end = new Pose(20, 0, 0);
        SimulationResult result = loop().followPath(r.getFollower(), r.getPlant(), r.getLocalizer(),
                r.getDrivetrain(), line(start, end), start);

        assertTrue(result.getSamples().size() > 10, "should record many loop samples");
        File out = new File("build/trace/straight_trace.csv");
        result.writeCsv(out);
        assertTrue(out.exists() && out.length() > 0, "trace CSV should be written");
    }
}
