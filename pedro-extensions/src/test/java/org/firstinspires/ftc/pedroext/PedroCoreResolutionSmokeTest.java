package org.firstinspires.ftc.pedroext;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathConstraints;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

/**
 * De-risking test for the whole project: proves the *real* Pedro Pathing core
 * artifact resolves from Maven Central and that its key types (the unmodified
 * Follower, the Drivetrain abstract class, and the Localizer interface) are on
 * the JVM classpath. If this passes, the headless follower-in-the-loop strategy
 * for Phases B and C is viable.
 */
class PedroCoreResolutionSmokeTest {

    @Test
    void pedroCoreTypesResolveAndLoad() {
        // Pure-Java geometry from core works on the JVM.
        Pose p = new Pose(3.0, 4.0, Math.PI / 2.0);
        assertEquals(3.0, p.getX(), 1e-9);
        assertEquals(4.0, p.getY(), 1e-9);
        assertEquals(Math.PI / 2.0, p.getHeading(), 1e-9);

        Vector v = new Vector(5.0, 0.0);
        assertEquals(5.0, v.getXComponent(), 1e-9);
        assertEquals(0.0, v.getYComponent(), 1e-9);

        // The real follower, drivetrain base class, and localizer interface are
        // present (forces them onto the compile + runtime classpath).
        assertNotNull(Follower.class.getName());
        assertNotNull(Drivetrain.class.getName());
        assertNotNull(Localizer.class.getName());
        assertNotNull(FollowerConstants.class.getName());
        assertNotNull(PathConstraints.class.getName());
    }
}
