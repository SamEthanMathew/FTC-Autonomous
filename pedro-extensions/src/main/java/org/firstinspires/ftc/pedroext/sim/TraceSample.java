package org.firstinspires.ftc.pedroext.sim;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

/**
 * One control-loop snapshot for the headless trace dump: ground-truth pose (from
 * the plant), the follower's estimated pose and target, the tracking error, and
 * the four correction vectors the follower produced. Immutable.
 */
public final class TraceSample {

    public final double t;            // seconds since start
    public final double dt;           // integration step used this loop

    // Ground truth (plant).
    public final double actualX, actualY, actualHeading;
    // Follower's pose estimate (== truth when there is no sensor noise).
    public final double estX, estY, estHeading;
    // Closest target point on the path.
    public final double targetX, targetY, targetHeading;

    public final double trackingError;   // |translational error| (inches)
    public final double headingError;    // radians
    public final double pathCompletion;  // 0..1
    public final double velocity;        // |follower velocity| (in/s)
    public final double voltage;         // bus voltage (volts)

    // Follower correction vectors (field-frame components).
    public final double correctiveX, correctiveY;
    public final double headingVecX, headingVecY;
    public final double driveX, driveY;
    public final double centripetalX, centripetalY;

    private TraceSample(double t, double dt,
                        double actualX, double actualY, double actualHeading,
                        double estX, double estY, double estHeading,
                        double targetX, double targetY, double targetHeading,
                        double trackingError, double headingError, double pathCompletion,
                        double velocity, double voltage,
                        double correctiveX, double correctiveY, double headingVecX, double headingVecY,
                        double driveX, double driveY, double centripetalX, double centripetalY) {
        this.t = t; this.dt = dt;
        this.actualX = actualX; this.actualY = actualY; this.actualHeading = actualHeading;
        this.estX = estX; this.estY = estY; this.estHeading = estHeading;
        this.targetX = targetX; this.targetY = targetY; this.targetHeading = targetHeading;
        this.trackingError = trackingError; this.headingError = headingError;
        this.pathCompletion = pathCompletion; this.velocity = velocity; this.voltage = voltage;
        this.correctiveX = correctiveX; this.correctiveY = correctiveY;
        this.headingVecX = headingVecX; this.headingVecY = headingVecY;
        this.driveX = driveX; this.driveY = driveY;
        this.centripetalX = centripetalX; this.centripetalY = centripetalY;
    }

    static TraceSample capture(double t, double dt, Follower follower, RobotPlant plant, double voltage) {
        Pose est = follower.getPose();
        Pose target = follower.getClosestPose() != null ? follower.getClosestPose().getPose() : new Pose();
        Vector err = follower.getTranslationalError();
        Vector corrective = follower.getCorrectiveVector();
        Vector headingVec = follower.getHeadingVector();
        Vector drive = follower.getDriveVector();
        Vector centripetal = follower.getCentripetalForceCorrection();
        return new TraceSample(t, dt,
                plant.getX(), plant.getY(), plant.getHeading(),
                est.getX(), est.getY(), est.getHeading(),
                target.getX(), target.getY(), target.getHeading(),
                err.getMagnitude(), follower.getHeadingError(), follower.getPathCompletion(),
                follower.getVelocity().getMagnitude(), voltage,
                corrective.getXComponent(), corrective.getYComponent(),
                headingVec.getXComponent(), headingVec.getYComponent(),
                drive.getXComponent(), drive.getYComponent(),
                centripetal.getXComponent(), centripetal.getYComponent());
    }

    /** CSV header matching {@link #toCsvRow()}. */
    public static String csvHeader() {
        return "t,dt,actualX,actualY,actualHeading,estX,estY,estHeading,targetX,targetY,targetHeading,"
                + "trackingError,headingError,pathCompletion,velocity,voltage,"
                + "correctiveX,correctiveY,headingVecX,headingVecY,driveX,driveY,centripetalX,centripetalY";
    }

    public String toCsvRow() {
        return t + "," + dt + "," + actualX + "," + actualY + "," + actualHeading + ","
                + estX + "," + estY + "," + estHeading + ","
                + targetX + "," + targetY + "," + targetHeading + ","
                + trackingError + "," + headingError + "," + pathCompletion + "," + velocity + "," + voltage + ","
                + correctiveX + "," + correctiveY + "," + headingVecX + "," + headingVecY + ","
                + driveX + "," + driveY + "," + centripetalX + "," + centripetalY;
    }
}
