package org.firstinspires.ftc.pedroext.sim;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.locks.LockSupport;

/**
 * Runs the <b>unmodified</b> Pedro {@link Follower} against the {@link RobotPlant}
 * headlessly and collects a {@link SimulationResult}.
 *
 * <h2>Why this runs in governed real time</h2>
 * Pedro's {@code PIDFController}, {@code FilteredPIDFController}, and
 * {@code PoseTracker} read {@code System.nanoTime()} directly (no injectable
 * clock), so their derivative/integral terms and velocity estimate use
 * wall-clock dt. A naive tight loop would feed them microsecond dt and destroy
 * their behavior. Instead each loop is paced to a target period, and — crucially
 * — the plant is integrated by the <em>measured</em> wall dt, so the plant and
 * the follower's internal clock stay consistent even under scheduling jitter.
 * The pace is configurable: ~6 ms (≈robot loop rate) for fidelity, smaller for
 * faster optimizer sweeps.
 */
public final class SimulationLoop {

    private final double targetLoopPeriodSeconds;
    private final double maxSimSeconds;
    private final int settleLoops;
    private final double maxDtSeconds;
    private final double settlePositionTolerance;  // inches
    private final double settleVelocityTolerance;  // in/s

    public SimulationLoop() {
        this(0.006, 20.0, 30, 0.05);
    }

    public SimulationLoop(double targetLoopPeriodSeconds, double maxSimSeconds, int settleLoops, double maxDtSeconds) {
        this(targetLoopPeriodSeconds, maxSimSeconds, settleLoops, maxDtSeconds, 0.5, 2.0);
    }

    /**
     * @param targetLoopPeriodSeconds  paced loop period (e.g. 0.006 for ~robot rate)
     * @param maxSimSeconds            wall-clock safety timeout for one follow
     * @param settleLoops              consecutive settled loops required to stop
     * @param maxDtSeconds             clamp on a single integration step (jitter guard)
     * @param settlePositionTolerance  tracking error (in) considered "settled"
     * @param settleVelocityTolerance  speed (in/s) considered "settled"
     */
    public SimulationLoop(double targetLoopPeriodSeconds, double maxSimSeconds, int settleLoops, double maxDtSeconds,
                          double settlePositionTolerance, double settleVelocityTolerance) {
        this.targetLoopPeriodSeconds = targetLoopPeriodSeconds;
        this.maxSimSeconds = maxSimSeconds;
        this.settleLoops = settleLoops;
        this.maxDtSeconds = maxDtSeconds;
        this.settlePositionTolerance = settlePositionTolerance;
        this.settleVelocityTolerance = settleVelocityTolerance;
    }

    public SimulationResult followPath(Follower follower, RobotPlant plant, SimulatedLocalizer localizer,
                                       SimulatedDrivetrain drivetrain, Path path, Pose start) {
        return followPath(follower, plant, localizer, drivetrain, path, start, Collections.emptyList());
    }

    public SimulationResult followPath(Follower follower, RobotPlant plant, SimulatedLocalizer localizer,
                                       SimulatedDrivetrain drivetrain, Path path, Pose start,
                                       List<ScheduledDisturbance> disturbances) {
        plant.setPose(start.getX(), start.getY(), start.getHeading());
        follower.setStartingPose(start);
        follower.followPath(path, true);
        return runLoop(follower, plant, localizer, drivetrain, disturbances);
    }

    public SimulationResult followPathChain(Follower follower, RobotPlant plant, SimulatedLocalizer localizer,
                                            SimulatedDrivetrain drivetrain, PathChain chain, Pose start) {
        return followPathChain(follower, plant, localizer, drivetrain, chain, start, Collections.emptyList());
    }

    public SimulationResult followPathChain(Follower follower, RobotPlant plant, SimulatedLocalizer localizer,
                                            SimulatedDrivetrain drivetrain, PathChain chain, Pose start,
                                            List<ScheduledDisturbance> disturbances) {
        plant.setPose(start.getX(), start.getY(), start.getHeading());
        follower.setStartingPose(start);
        follower.followPath(chain, true);
        return runLoop(follower, plant, localizer, drivetrain, disturbances);
    }

    private SimulationResult runLoop(Follower follower, RobotPlant plant, SimulatedLocalizer localizer,
                                     SimulatedDrivetrain drivetrain, List<ScheduledDisturbance> disturbances) {
        List<TraceSample> samples = new ArrayList<>();
        long periodNanos = (long) (targetLoopPeriodSeconds * 1e9);
        long startNanos = System.nanoTime();
        long prevNanos = startNanos;
        boolean timedOut = false;
        int settle = 0;

        while (true) {
            LockSupport.parkNanos(periodNanos);

            long now = System.nanoTime();
            double dt = Math.min((now - prevNanos) / 1e9, maxDtSeconds);
            prevNanos = now;
            double tSec = (now - startNanos) / 1e9;

            for (ScheduledDisturbance d : disturbances) {
                d.applyIfDue(tSec, plant);
            }

            localizer.update();
            follower.update();
            plant.step(dt);

            TraceSample sample = TraceSample.capture(tSec, dt, follower, plant, drivetrain.getVoltage());
            samples.add(sample);

            // Stop only once the follower has finished AND the robot has actually
            // settled on the (held) endpoint — so endpoint overshoot is allowed to
            // recover before we record the final pose.
            boolean settled = !follower.isBusy()
                    && sample.trackingError < settlePositionTolerance
                    && sample.velocity < settleVelocityTolerance;
            if (settled) {
                if (++settle >= settleLoops) {
                    break;
                }
            } else {
                settle = 0;
            }
            if (tSec > maxSimSeconds) {
                timedOut = true;
                break;
            }
        }

        double duration = (System.nanoTime() - startNanos) / 1e9;
        return new SimulationResult(samples, plant.getX(), plant.getY(), plant.getHeading(), duration, timedOut);
    }
}
