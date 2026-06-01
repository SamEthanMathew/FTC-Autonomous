package org.firstinspires.ftc.pedroext.fusion;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

/**
 * A 3-DOF (x, y, heading) Extended Kalman Filter that fuses odometry with absolute
 * pose measurements (e.g. AprilTag fixes). The measurement model is the identity
 * (a fix directly observes the pose), so the update is a linear Kalman update,
 * with heading innovation wrapped to (-π, π].
 *
 * <p><b>Smooth corrections (no pose discontinuity):</b> the per-update correction
 * is magnitude-limited (position and heading caps). A large innovation — e.g. the
 * first good tag after the robot was shoved — is then applied gradually over
 * several loops instead of snapping, so the follower never sees a jump. This is a
 * deliberate smoothing heuristic on top of the strict KF gain.
 */
public final class PoseEKF {

    private final double[] state = new double[3]; // x, y, heading
    private RealMatrix covariance;

    public PoseEKF(double x, double y, double heading, double initialVariance) {
        state[0] = x;
        state[1] = y;
        state[2] = normalizeAngle(heading);
        covariance = scaledIdentity(initialVariance);
    }

    public double getX() { return state[0]; }
    public double getY() { return state[1]; }
    public double getHeading() { return state[2]; }
    public RealMatrix getCovariance() { return covariance.copy(); }

    /** Total position variance (trace of the x,y block) — a scalar uncertainty measure. */
    public double getPositionVariance() {
        return covariance.getEntry(0, 0) + covariance.getEntry(1, 1);
    }

    public void setState(double x, double y, double heading, double variance) {
        state[0] = x;
        state[1] = y;
        state[2] = normalizeAngle(heading);
        covariance = scaledIdentity(variance);
    }

    /**
     * Prediction from an odometry displacement (field frame). The displacement is
     * added to the state and the process-noise covariance {@code Q} grows the
     * uncertainty.
     */
    public void predict(double dx, double dy, double dHeading, double[] qDiagonal) {
        state[0] += dx;
        state[1] += dy;
        state[2] = normalizeAngle(state[2] + dHeading);
        covariance = covariance.add(diagonal(qDiagonal));
    }

    /**
     * Measurement update from an absolute pose fix with diagonal measurement
     * covariance {@code R}. The applied correction is capped for smoothness.
     *
     * @param maxPositionCorrection max position move applied this update (inches)
     * @param maxHeadingCorrection  max heading move applied this update (radians)
     */
    public void update(double zx, double zy, double zHeading, double[] rDiagonal,
                       double maxPositionCorrection, double maxHeadingCorrection) {
        double[] innovation = {
                zx - state[0],
                zy - state[1],
                normalizeAngle(zHeading - state[2]),
        };

        RealMatrix r = diagonal(rDiagonal);
        RealMatrix s = covariance.add(r);                       // S = P + R (H = I)
        RealMatrix sInv = new LUDecomposition(s).getSolver().getInverse();
        RealMatrix k = covariance.multiply(sInv);               // K = P S^-1

        double[] correction = k.operate(innovation);

        // Smoothing: cap the correction so a big innovation is applied gradually.
        double posMag = Math.hypot(correction[0], correction[1]);
        if (posMag > maxPositionCorrection && posMag > 0) {
            double scale = maxPositionCorrection / posMag;
            correction[0] *= scale;
            correction[1] *= scale;
        }
        correction[2] = clamp(correction[2], -maxHeadingCorrection, maxHeadingCorrection);

        state[0] += correction[0];
        state[1] += correction[1];
        state[2] = normalizeAngle(state[2] + correction[2]);

        RealMatrix identity = MatrixUtils.createRealIdentityMatrix(3);
        covariance = identity.subtract(k).multiply(covariance);
    }

    private static RealMatrix diagonal(double[] d) {
        double[][] m = new double[3][3];
        for (int i = 0; i < 3; i++) {
            m[i][i] = d[i];
        }
        return new Array2DRowRealMatrix(m, false);
    }

    private static RealMatrix scaledIdentity(double v) {
        return MatrixUtils.createRealIdentityMatrix(3).scalarMultiply(v);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    static double normalizeAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }
}
