package org.firstinspires.ftc.teamcode.tuning;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.pedroext.tuning.GainSet;
import org.firstinspires.ftc.pedroext.tuning.OptimizationResult;
import org.firstinspires.ftc.pedroext.tuning.Optimizer;
import org.firstinspires.ftc.pedroext.tuning.TrajectoryEvaluator;
import org.firstinspires.ftc.pedroext.tuning.TwiddleOptimizer;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;

/**
 * Phase C2 (on-robot) — closed-loop auto-PID using the SAME optimizer as the
 * offline/simulator tuner (proven in the JVM tests), just behind an on-robot
 * {@link TrajectoryEvaluator}. Lightweight {@link TwiddleOptimizer} is used since
 * it runs comfortably on the Robot Controller.
 *
 * <p>HARDWARE-VALIDATION REQUIRED. Each candidate evaluation: writes the gains into
 * the constants, rebuilds the follower, drives a there-and-back validation path,
 * and scores tracking RMSE + endpoint error. Every trial's gains + cost are
 * appended to a CSV on device storage so a run can be inspected or resumed.
 *
 * <p>NEEDS CLEAR SPACE and supervision. The validation path returns the robot to
 * its start so trials repeat in place; verify this on YOUR field before trusting
 * it. The on-robot RMSE measurement is unverified here.
 */
@Autonomous(name = "Auto-Tune PID (Pedro)", group = "tuning")
public class AutoTuneOpMode extends LinearOpMode {

    public static double PATH_LENGTH = 40.0;       // inches out (and back)
    public static int MAX_TRIALS = 25;
    public static double TRIAL_TIMEOUT_SECONDS = 8.0;
    public static double ENDPOINT_WEIGHT = 0.5;

    private final GainSet startGains = GainSet.pedroDefaults();
    private File trialLog;

    @Override
    public void runOpMode() {
        trialLog = new File(AppUtil.FIRST_FOLDER, "pedro_autotune_trials.csv");
        writeTrialHeader();

        telemetry.addLine("Auto-Tune: clear ~" + (PATH_LENGTH + 24) + " in ahead. Robot tunes itself.");
        telemetry.addLine("Press START. Each trial drives out-and-back; trials logged to " + trialLog.getName());
        telemetry.update();
        waitForStart();

        TrajectoryEvaluator evaluator = new OnRobotEvaluator();
        Optimizer optimizer = new TwiddleOptimizer(0.3, 0.03, MAX_TRIALS);
        OptimizationResult result = optimizer.optimize(startGains, evaluator);

        telemetry.addLine("Best gains: " + result.getBestGains());
        telemetry.addData("Best cost", result.getBestCost());
        telemetry.addData("Trials", result.getEvaluations());
        telemetry.addLine("Apply these to Constants (translational/heading/drive PIDF + centripetal).");
        telemetry.update();
        while (opModeIsActive()) {
            sleep(100);
        }
    }

    /** Scores a candidate by running the real follower on a there-and-back path. */
    private final class OnRobotEvaluator implements TrajectoryEvaluator {
        private int trial = 0;

        @Override
        public double cost(GainSet gains) {
            if (!opModeIsActive()) {
                return Double.MAX_VALUE;
            }
            // Apply candidate gains to the team's constants and rebuild the follower.
            gains.clampedToBounds().applyTo(Constants.followerConstants);
            Follower follower = Constants.createFollower(hardwareMap);

            Pose start = new Pose(0, 0, 0);
            follower.setStartingPose(start);
            PathChain path = follower.pathBuilder()
                    .addPath(straight(new Pose(0, 0, 0), new Pose(PATH_LENGTH, 0, 0)))
                    .addPath(straight(new Pose(PATH_LENGTH, 0, 0), new Pose(0, 0, 0)))
                    .build();
            follower.followPath(path, true);

            ArrayList<Double> errors = new ArrayList<>();
            ElapsedTime timer = new ElapsedTime();
            while (opModeIsActive() && follower.isBusy() && timer.seconds() < TRIAL_TIMEOUT_SECONDS) {
                follower.update();
                errors.add(follower.getTranslationalError().getMagnitude());
            }
            follower.breakFollowing();

            double rmse = rms(errors);
            double endpointError = Math.hypot(follower.getPose().getX() - start.getX(),
                    follower.getPose().getY() - start.getY());
            double cost = rmse + ENDPOINT_WEIGHT * endpointError;

            logTrial(++trial, gains, cost);
            telemetry.addData("trial " + trial, "cost=%.3f rmse=%.3f", cost, rmse);
            telemetry.update();
            return cost;
        }
    }

    private static Path straight(Pose a, Pose b) {
        Path p = new Path(new BezierLine(a, b));
        p.setConstantHeadingInterpolation(0);
        return p;
    }

    private static double rms(ArrayList<Double> values) {
        if (values.isEmpty()) {
            return 0.0;
        }
        double sumSq = 0.0;
        for (double v : values) {
            sumSq += v * v;
        }
        return Math.sqrt(sumSq / values.size());
    }

    private void writeTrialHeader() {
        try (FileWriter w = new FileWriter(trialLog, false)) {
            w.write("trial,translationalP,translationalD,headingP,headingD,driveP,centripetalScaling,cost\n");
        } catch (Exception ignored) {
            // best-effort logging
        }
    }

    private void logTrial(int trial, GainSet g, double cost) {
        double[] a = g.toArray();
        try (FileWriter w = new FileWriter(trialLog, true)) {
            w.write(trial + "," + a[0] + "," + a[1] + "," + a[2] + "," + a[3] + "," + a[4] + "," + a[5] + "," + cost + "\n");
        } catch (Exception ignored) {
            // best-effort logging
        }
    }
}
