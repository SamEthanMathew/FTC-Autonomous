package org.firstinspires.ftc.pedroext.tuning;

/**
 * A black-box optimizer over {@link GainSet} driven by a {@link TrajectoryEvaluator}.
 * The same optimizer works against the simulator (offline) or the robot (on-robot)
 * because both are just {@code TrajectoryEvaluator}s.
 */
public interface Optimizer {

    OptimizationResult optimize(GainSet initial, TrajectoryEvaluator evaluator);
}
