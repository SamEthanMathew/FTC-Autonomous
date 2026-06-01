package org.firstinspires.ftc.teamcode.tuning;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.pedroext.tuning.CalibrationResult;
import org.firstinspires.ftc.pedroext.tuning.CalibrationStore;
import org.firstinspires.ftc.pedroext.tuning.RobustStatistics;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;

/**
 * Phase C1 — one-shot self-calibration harness.
 *
 * <p>HARDWARE-VALIDATION REQUIRED. Runs the measurement-based tuners back-to-back
 * (forward velocity, lateral velocity, forward zero-power accel, lateral zero-power
 * accel), {@code RUNS} times each (alternating direction so the robot stays put),
 * rejects outliers and averages via the unit-tested
 * {@link org.firstinspires.ftc.pedroext.tuning.RobustStatistics}, then writes the
 * results to device storage via {@link CalibrationStore} — no hand-transcribing.
 *
 * <p>Guard rails: requires a minimum run distance, and warns (via telemetry +
 * stored note) if a velocity measurement is still climbing across runs, which
 * means the robot never reached terminal velocity (the classic accuracy trap).
 *
 * <p>NEEDS CLEAR SPACE: the robot runs at full power for {@code VELOCITY_DISTANCE}
 * inches in each direction and coasts after cutting power. The measurement math
 * here (velocity magnitude / coast deceleration) is unverified on hardware.
 */
@Autonomous(name = "Self-Calibration (Pedro)", group = "tuning")
public class SelfCalibrationOpMode extends LinearOpMode {

    public static int RUNS = 5;
    public static double VELOCITY_DISTANCE = 48.0;   // inches to accelerate over
    public static double MIN_DISTANCE = 24.0;        // guard rail: minimum travel
    public static double ZPA_TRIGGER_VELOCITY = 30.0; // in/s at which to cut power
    public static int VELOCITY_WINDOW = 10;          // recent samples to average
    public static double OUTLIER_MAD_MULTIPLIER = 2.5;
    public static double CLIMBING_TOLERANCE = 0.05;  // 5% rise across runs => warn

    private Follower follower;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        telemetry.addLine("Self-Calibration: ensure ~6 ft of clear space ahead and to the side.");
        telemetry.addLine("Press START to measure velocity + zero-power acceleration (" + RUNS + " runs each).");
        telemetry.update();
        waitForStart();

        double[] forwardVel = new double[RUNS];
        double[] lateralVel = new double[RUNS];
        double[] forwardZpa = new double[RUNS];
        double[] lateralZpa = new double[RUNS];

        for (int i = 0; i < RUNS && opModeIsActive(); i++) {
            double sign = (i % 2 == 0) ? 1.0 : -1.0;  // alternate direction to stay in place
            forwardVel[i] = measureVelocity(true, sign);
            lateralVel[i] = measureVelocity(false, sign);
            forwardZpa[i] = measureZeroPowerAcceleration(true, sign);
            lateralZpa[i] = measureZeroPowerAcceleration(false, sign);
            telemetry.addData("run " + (i + 1) + "/" + RUNS, "fV=%.1f lV=%.1f fZPA=%.1f lZPA=%.1f",
                    forwardVel[i], lateralVel[i], forwardZpa[i], lateralZpa[i]);
            telemetry.update();
        }

        CalibrationResult result = new CalibrationResult(
                RobustStatistics.robustMean(forwardVel, OUTLIER_MAD_MULTIPLIER),
                RobustStatistics.robustMean(lateralVel, OUTLIER_MAD_MULTIPLIER),
                RobustStatistics.robustMean(forwardZpa, OUTLIER_MAD_MULTIPLIER),
                RobustStatistics.robustMean(lateralZpa, OUTLIER_MAD_MULTIPLIER),
                RUNS);

        boolean climbing = RobustStatistics.isStillClimbing(forwardVel, CLIMBING_TOLERANCE)
                || RobustStatistics.isStillClimbing(lateralVel, CLIMBING_TOLERANCE);

        String savedPath = save(result, climbing);

        telemetry.addLine(result.toConstantsSnippet());
        if (climbing) {
            telemetry.addLine("WARNING: velocity still climbing across runs — increase VELOCITY_DISTANCE.");
        }
        telemetry.addData("saved to", savedPath);
        telemetry.update();
        while (opModeIsActive()) {
            sleep(100);
        }
    }

    /** Drives at full power along an axis until {@code VELOCITY_DISTANCE}, averaging recent speed. */
    private double measureVelocity(boolean forwardAxis, double sign) {
        Pose start = follower.getPose();
        follower.startTeleopDrive(false);
        ArrayList<Double> window = new ArrayList<>();
        double traveled = 0.0;
        while (opModeIsActive() && traveled < VELOCITY_DISTANCE) {
            follower.update();
            if (forwardAxis) {
                follower.setTeleOpDrive(sign, 0, 0, true);
            } else {
                follower.setTeleOpDrive(0, sign, 0, true);
            }
            window.add(follower.getVelocity().getMagnitude());
            if (window.size() > VELOCITY_WINDOW) {
                window.remove(0);
            }
            traveled = distance(start, follower.getPose());
        }
        stopDrive();
        if (traveled < MIN_DISTANCE) {
            telemetry.addLine("WARNING: run distance " + traveled + " < MIN_DISTANCE; result unreliable.");
        }
        return mean(window);
    }

    /** Accelerates to {@code ZPA_TRIGGER_VELOCITY}, cuts power, and averages the coast deceleration. */
    private double measureZeroPowerAcceleration(boolean forwardAxis, double sign) {
        follower.startTeleopDrive(false);
        // Spin up.
        while (opModeIsActive() && follower.getVelocity().getMagnitude() < ZPA_TRIGGER_VELOCITY) {
            follower.update();
            if (forwardAxis) {
                follower.setTeleOpDrive(sign, 0, 0, true);
            } else {
                follower.setTeleOpDrive(0, sign, 0, true);
            }
        }
        // Cut power and record deceleration while coasting.
        follower.setTeleOpDrive(0, 0, 0, true);
        ArrayList<Double> decels = new ArrayList<>();
        ElapsedTime timer = new ElapsedTime();
        double previous = follower.getVelocity().getMagnitude();
        double minVelocity = follower.getConstraints().getVelocityConstraint();
        while (opModeIsActive() && follower.getVelocity().getMagnitude() > minVelocity) {
            follower.update();
            double dt = timer.seconds();
            if (dt > 0) {
                double current = follower.getVelocity().getMagnitude();
                decels.add((current - previous) / dt); // negative while decelerating
                previous = current;
            }
            timer.reset();
        }
        stopDrive();
        return mean(decels);
    }

    private void stopDrive() {
        follower.setTeleOpDrive(0, 0, 0, true);
        follower.update();
        follower.breakFollowing();
    }

    private static double distance(Pose a, Pose b) {
        return Math.hypot(a.getX() - b.getX(), a.getY() - b.getY());
    }

    private static double mean(ArrayList<Double> values) {
        if (values.isEmpty()) {
            return 0.0;
        }
        double sum = 0.0;
        for (double v : values) {
            sum += v;
        }
        return sum / values.size();
    }

    private String save(CalibrationResult result, boolean climbing) {
        File out = new File(AppUtil.FIRST_FOLDER, "pedro_calibration.properties");
        try (FileWriter writer = new FileWriter(out)) {
            CalibrationStore.write(writer, result);
            return out.getAbsolutePath();
        } catch (Exception e) {
            telemetry.addData("save failed", e.getMessage());
            return "(not saved)";
        }
    }
}
