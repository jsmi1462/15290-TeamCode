package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Robot hardware and tuning numbers live here and nowhere else.
 *
 * HOW TO FILL THIS IN (mentor, once per robot):
 *   1. Run the Tuning OpMode from the Driver Station.
 *   2. Each tuner prints a code block. Paste it over the matching block below.
 *   3. Set TUNED = true.
 *
 * Every value below is a PLACEHOLDER so the project compiles. None of them are tuned
 * numbers for any real robot. Until TUNED is true, create() refuses to build a follower.
 */
public class Constants {

    /** Flip to true only after the three config blocks below hold real tuner output. */
    public static boolean TUNED = false;

    // ---- PASTE MecanumTuner OUTPUT HERE ------------------------------------------------
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // ---- PASTE PinpointTuner OUTPUT HERE -----------------------------------------------
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(0.0);
        c.yPodOffset.set(0.0);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // ---- PASTE ForesightTuner OUTPUT HERE ----------------------------------------------
    // Placeholder numbers only. They exist so the validators (which require positive
    // values) accept the config at class load.
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.1);
                Controller secondaryTranslationalForward = Controller.proportional(0.1);
                Controller primaryTranslationalLateral = Controller.proportional(0.1);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.1));
                c.brake.set(Controller.proportionalFeedforward(0.1));

                c.headingFeedback.set(Controller.proportional(0.1));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.1, 0.1));

                c.linearBrakeCoefficients.set(Matrix.diag(0.1, 0.1));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.1, 0.1));

                c.maxAchievableForwardVelocity.set(1.0);
                c.maxAchievableStrafeVelocity.set(1.0);
                c.naturalForwardDeceleration.set(1.0);
                c.naturalStrafeDeceleration.set(1.0);
            }
    );

    /** Builds the follower every OpMode uses. Students never edit this method. */
    public static Follower create(HardwareMap h) {
        if (!TUNED) {
            throw new IllegalStateException(
                    "Constants.TUNED is false. Run the Tuning OpMode, paste its output into "
                            + "Constants.java, then set TUNED = true.");
        }
        Localizer localizer = new PinpointLocalizer(h, localizerConfig);
        Drivetrain drivetrain = new Mecanum(h, drivetrainConfig);
        Algorithm algorithm = new Foresight(foresightConfig);
        return new Follower(localizer, drivetrain, algorithm);
    }
}
