package org.firstinspires.ftc.pedroext.tuning;

/**
 * The single seam that decouples the optimizer from where it runs (§4 of the
 * brief): a candidate {@link GainSet} in, a scalar cost out (lower is better).
 *
 * <p>Implemented twice with the <b>same</b> optimizer code on top:
 * <ul>
 *   <li>{@link SimTrajectoryEvaluator} — offline, free, runs the real follower
 *       against the Phase B simulator (used by the JVM tests).</li>
 *   <li>an on-robot evaluator (in TeamCode) that runs a fixed validation path on
 *       the real robot and reports tracking RMSE — {@code HARDWARE-VALIDATION
 *       REQUIRED}.</li>
 * </ul>
 */
public interface TrajectoryEvaluator {

    /**
     * Evaluates a candidate gain set and returns a scalar cost (tracking RMSE
     * plus penalties). Must return a large finite value for diverged/failed runs
     * rather than NaN/Infinity, so optimizers can compare it.
     */
    double cost(GainSet gains);
}
