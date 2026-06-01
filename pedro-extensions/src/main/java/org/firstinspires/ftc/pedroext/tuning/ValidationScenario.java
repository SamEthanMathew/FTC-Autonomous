package org.firstinspires.ftc.pedroext.tuning;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import java.util.Arrays;
import java.util.function.Supplier;

/**
 * A fixed validation path used to score candidate gains. The path is rebuilt per
 * evaluation (via the supplier) so no path-following state leaks between trials.
 */
public final class ValidationScenario {

    private final Supplier<Path> pathFactory;
    private final Pose start;
    private final Pose end;

    public ValidationScenario(Supplier<Path> pathFactory, Pose start, Pose end) {
        this.pathFactory = pathFactory;
        this.start = start;
        this.end = end;
    }

    /**
     * A short curved path with a 90° heading change — exercises the translational,
     * heading, drive, and centripetal terms while staying brief (the sim paces in
     * real time, so optimizer sweeps need short paths).
     */
    public static ValidationScenario defaultCurve() {
        Pose start = new Pose(0, 0, 0);
        Pose end = new Pose(20, 20, Math.PI / 2.0);
        Supplier<Path> factory = () -> {
            Path path = new Path(new BezierCurve(Arrays.asList(new Pose(0, 0), new Pose(20, 0), new Pose(20, 20))));
            path.setLinearHeadingInterpolation(0, Math.PI / 2.0);
            return path;
        };
        return new ValidationScenario(factory, start, end);
    }

    /**
     * The default auto-tuning scenario: a long quarter-turn curve with the heading
     * following the tangent. Sustained curvature continuously demands translational,
     * heading, drive, and centripetal authority, so gain quality shows up clearly as
     * tracking error + progress over a fixed window (no end-overshoot artifact since
     * the window does not reach the end).
     */
    public static ValidationScenario tuningCurve() {
        Pose start = new Pose(0, 0, 0);
        Pose end = new Pose(40, 40, Math.PI / 2.0);
        Supplier<Path> factory = () -> {
            Path path = new Path(new BezierCurve(Arrays.asList(new Pose(0, 0), new Pose(40, 0), new Pose(40, 40))));
            path.setTangentHeadingInterpolation();
            return path;
        };
        return new ValidationScenario(factory, start, end);
    }

    /** A shorter curve for fast optimizer sweeps in tests (still curved + a turn). */
    public static ValidationScenario shortCurve() {
        Pose start = new Pose(0, 0, 0);
        Pose end = new Pose(14, 14, Math.PI / 2.0);
        Supplier<Path> factory = () -> {
            Path path = new Path(new BezierCurve(Arrays.asList(new Pose(0, 0), new Pose(14, 0), new Pose(14, 14))));
            path.setLinearHeadingInterpolation(0, Math.PI / 2.0);
            return path;
        };
        return new ValidationScenario(factory, start, end);
    }

    /**
     * A short straight path the robot must converge onto from an offset start
     * (laterally displaced and rotated). Tracking RMSE is then dominated by how
     * quickly and cleanly the follower corrects onto the line — a fast, low-noise,
     * gain-sensitive signal ideal for the auto-tuner (and quick in real time).
     */
    public static ValidationScenario offsetStraight() {
        Pose start = new Pose(0, 6, 0.15);             // 6 in + 0.15 rad off the path
        Pose end = new Pose(80, 0, 0);
        Supplier<Path> factory = () -> {
            // Long line so a fixed-time window captures the convergence transient and
            // steady tracking, never the robot reaching (and overshooting) the end.
            Path path = new Path(new BezierLine(new Pose(0, 0, 0), new Pose(80, 0, 0)));
            path.setConstantHeadingInterpolation(0);
            return path;
        };
        return new ValidationScenario(factory, start, end);
    }

    public Path newPath() { return pathFactory.get(); }
    public Pose getStart() { return start; }
    public Pose getEnd() { return end; }
}
