package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing configuration for this robot (mecanum + goBILDA Pinpoint).
 *
 * <p>HARDWARE-VALIDATION REQUIRED: this file is compiled inside an FTC SDK
 * project (Android SDK present); it is NOT compiled in the headless dev
 * environment. The motor/encoder names, directions, and pod offsets below are
 * placeholders — confirm them against your robot configuration.
 *
 * <p>Phase A: voltage compensation is ENABLED here. Pedro 2.1.2 normalizes motor
 * output by measured bus voltage internally (see {@code Mecanum}); the pure-Java
 * port of that exact math, used by the offline tooling, is unit-tested in
 * {@code org.firstinspires.ftc.pedroext.voltage.VoltageCompensationModel}.
 *
 * <p>The velocity / zero-power / mass constants below are Pedro library defaults
 * as a safe starting point; Phase C's self-calibration OpMode overwrites them
 * with measured, voltage-normalized values.
 */
public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.65)                              // kg — replace via self-cal (Phase C)
            .forwardZeroPowerAcceleration(-41.278)    // in/s^2 — replace via self-cal
            .lateralZeroPowerAcceleration(-59.7819);  // in/s^2 — replace via self-cal

    public static MecanumConstants driveConstants = new MecanumConstants()
            .xVelocity(81.34)   // in/s — replace via self-cal (Phase C)
            .yVelocity(65.43)   // in/s — replace via self-cal
            // --- Phase A: battery-independent output -----------------------
            .useVoltageCompensation(true)
            .nominalVoltage(12.0)
            .staticFrictionCoefficient(0.1)
            // --- hardware mapping (PLACEHOLDERS — confirm on robot) --------
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftRear")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear");

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .hardwareMapName("pinpoint")
            .distanceUnit(DistanceUnit.INCH)
            .forwardPodY(1.0)    // PLACEHOLDER offset (in) — confirm on robot
            .strafePodX(-2.5)    // PLACEHOLDER offset (in) — confirm on robot
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
