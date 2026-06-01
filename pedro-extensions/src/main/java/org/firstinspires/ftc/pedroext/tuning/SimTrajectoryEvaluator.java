package org.firstinspires.ftc.pedroext.tuning;

import com.pedropathing.follower.FollowerConstants;
import org.firstinspires.ftc.pedroext.sim.SimulatedRobot;
import org.firstinspires.ftc.pedroext.sim.SimulationLoop;
import org.firstinspires.ftc.pedroext.sim.SimulationResult;

/**
 * Offline {@link TrajectoryEvaluator} that scores a gain set by running the real
 * Pedro follower against the Phase B simulator on a {@link ValidationScenario}.
 *
 * <p>Cost = tracking RMSE + a penalty for the worst tracking excursion
 * (overshoot/oscillation) + a penalty for missing the endpoint. A diverged or
 * timed-out run returns a large finite cost so optimizers can still rank it.
 */
public final class SimTrajectoryEvaluator implements TrajectoryEvaluator {

    /** Cost returned for a run that timed out or produced non-finite numbers. */
    public static final double FAILURE_COST = 1.0e6;

    private final ValidationScenario scenario;
    private final SimulationLoop loop;
    private final double progressWeight;
    private final double overshootWeight;

    public SimTrajectoryEvaluator(ValidationScenario scenario) {
        // Fixed 1.2 s window per evaluation: uniform, short, no timeout cliffs.
        this(scenario, SimulationLoop.fixedWindow(1.2, 0.004, 0.05), 6.0, 0.1);
    }

    /**
     * @param progressWeight  penalty per unit of un-completed path (discourages the
     *                        degenerate "go slow to minimize RMSE" solution)
     * @param overshootWeight penalty on the worst tracking excursion (overshoot/oscillation)
     */
    public SimTrajectoryEvaluator(ValidationScenario scenario, SimulationLoop loop,
                                  double progressWeight, double overshootWeight) {
        this.scenario = scenario;
        this.loop = loop;
        this.progressWeight = progressWeight;
        this.overshootWeight = overshootWeight;
    }

    @Override
    public double cost(GainSet gains) {
        FollowerConstants constants = gains.clampedToBounds().toConstants();
        SimulatedRobot robot = SimulatedRobot.create(constants);
        SimulationResult result;
        try {
            result = loop.followPath(robot.getFollower(), robot.getPlant(), robot.getLocalizer(),
                    robot.getDrivetrain(), scenario.newPath(), scenario.getStart());
        } catch (RuntimeException ex) {
            return FAILURE_COST;
        }
        if (robot.getLocalizer().isNAN()) {
            return FAILURE_COST;
        }
        // Tracking accuracy (RMSE) + a progress term so slow gains can't win by
        // crawling + an overshoot/oscillation penalty.
        double cost = result.getRmseTrackingError()
                + progressWeight * (1.0 - result.getFinalPathCompletion())
                + overshootWeight * result.getMaxTrackingError();
        return Double.isFinite(cost) ? cost : FAILURE_COST;
    }
}
