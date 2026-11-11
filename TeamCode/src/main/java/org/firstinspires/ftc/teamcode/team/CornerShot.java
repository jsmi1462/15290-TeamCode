package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * CORNER SHOT: back into a corner, press one button, and the robot drives itself to the shooting spot and shoots.
 *
 * WHY THIS WORKS WITHOUT A CAMERA
 *   The corner is the one place on the field where we know EXACTLY where the robot is. When both distance
 *   sensors read cornerN inches, the robot is tucked into the corner, so we set the Pinpoint to x=0, y=0,
 *   angle=0. From there, the shooting spot is always the same distance away.
 *
 * WHICH CORNERS
 *   The sensors are on the robot's back-right corner, so this works in any field corner where the walls end up
 *   BEHIND the robot and to its RIGHT (for us: the front-left and back-right field corners).
 *   For the other two corners the walls are behind and to the LEFT: you need a sensor on the left side, and
 *   shotLeftIn has the opposite sign.
 *
 * THE STEPS (each one uses the same "start, drive, check, stop" idea as BaseAuto)
 *   1. resetPose()           tell the Pinpoint "you are at 0, 0, 0"
 *   2. driveToShootingSpot() forward, then sideways, then turn. The flywheel spins up while we drive.
 *   3. shoot()               wait for the flywheel to be at speed, then feed with the intake and windmill
 *
 * The same class works in TeleOp (CornerShotTeleOp) and Autonomous (CornerShotAuto).
 * In TeleOp, driver 1 can cancel at any time by moving a stick or pressing B.
 */
@Config
public class CornerShot {

    // ---- The shooting spot, measured from the corner. Change these live in FTC Dashboard. ----
    // How to measure: in CornerShotTeleOp, back into the corner until telemetry says "RESET DONE",
    // drive by hand to the best shooting spot, and copy the three "from corner" numbers here.
    public static double shotForwardIn = 0;    // inches forward from the corner
    public static double shotLeftIn = 0;       // inches LEFT from the corner (negative = right)
    public static double shotHeadingDeg = 0;   // the angle to face when shooting (Pinpoint degrees)

    // ---- How it drives ----
    public static double fastPower = 0.8;      // drive power for most of the trip
    public static double slowPower = 0.3;      // drive power for the last few inches, so we don't overshoot
    public static double slowZoneIn = 6;       // switch to slowPower this many inches before the end
    public static double toleranceIn = 0.5;    // "close enough" for driving
    public static double headingKp = 0.03;     // turn power per degree of error (same idea as snap turn)
    public static double maxTurnPower = 0.5;
    public static double toleranceDeg = 1.5;   // "close enough" for turning
    public static double moveTimeoutS = 5;     // give up on one step if it takes longer than this

    // ---- How it shoots ----
    public static double shooterSetting = 0.7; // 0 to 1, same scale as the Y/X shooter setting in DriveMode
    public static double feedMs = 3000;        // how long to run the intake and windmill to feed balls

    private final LinearOpMode opMode;
    private final DcMotor frontLeft, backLeft, frontRight, backRight, intake, windmill;
    private final ShooterFlywheel flywheel;
    private final GoBildaPinpointDriver pinpoint;
    private final DistanceSensor distBack, distRight;
    private final ElapsedTime cornerTimer = new ElapsedTime();

    public CornerShot(LinearOpMode opMode) {
        this.opMode = opMode;
        frontLeft = opMode.hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = opMode.hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = opMode.hardwareMap.get(DcMotor.class, "frontRight");
        backRight = opMode.hardwareMap.get(DcMotor.class, "backRight");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        for (DcMotor m : new DcMotor[]{frontLeft, backLeft, frontRight, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        DcMotorEx shooter1 = opMode.hardwareMap.get(DcMotorEx.class, "shooter1");
        DcMotorEx shooter2 = opMode.hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter1.setDirection(DcMotor.Direction.FORWARD);
        shooter2.setDirection(DcMotor.Direction.REVERSE);   // same as DriveMode
        flywheel = new ShooterFlywheel(shooter1, shooter2);
        flywheel.init();
        intake = opMode.hardwareMap.get(DcMotor.class, "intake");
        windmill = opMode.hardwareMap.get(DcMotor.class, "windmill");

        pinpoint = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(   // with these, forward reads NEGATIVE x and left reads NEGATIVE y
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        distBack = opMode.hardwareMap.get(DistanceSensor.class, "distBack");
        distRight = opMode.hardwareMap.get(DistanceSensor.class, "distRight");
    }

    // =====================================================================
    //  Step 0: are we in the corner?
    // =====================================================================
    /** True when BOTH sensors have read about cornerN inches for cornerHoldMs. Call every loop. */
    public boolean inCorner() {
        double back = distBack.getDistance(DistanceUnit.INCH);
        double right = distRight.getDistance(DistanceUnit.INCH);
        // A sensor never reads exactly 2.000, so we ask "is it CLOSE to 2?" instead of "== 2".
        boolean closeNow = Math.abs(back - TuningConfig.cornerN) <= TuningConfig.cornerTolIn
                && Math.abs(right - TuningConfig.cornerN) <= TuningConfig.cornerTolIn;
        if (!closeNow) {
            cornerTimer.reset();   // bumped out of the corner: start waiting again
        }
        opMode.telemetry.addData("Distance back / right (in)", "%.1f / %.1f", back, right);
        return closeNow && cornerTimer.milliseconds() >= TuningConfig.cornerHoldMs;
    }

    // =====================================================================
    //  Step 1: we KNOW where we are, so tell the Pinpoint.
    // =====================================================================
    public void resetPose() {
        // setPosition is instant. resetPosAndIMU() would also work, but it needs the robot perfectly
        // still for about 0.25 s while the IMU recalibrates.
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        pinpoint.update();
    }

    // =====================================================================
    //  Step 2: drive there. Returns false if it was cancelled or failed.
    // =====================================================================
    public boolean driveToShootingSpot() {
        boolean ok = move("Forward", 1, 0, Math.abs(shotForwardIn), shotForwardIn < 0)
                && move("Sideways", 0, 1, Math.abs(shotLeftIn), shotLeftIn < 0)
                && turnTo(shotHeadingDeg);
        stopWheels();
        return ok;
    }

    /**
     * Drive in a straight line until we have gone `inches` from where this step started.
     * forwardDir / leftDir pick the direction (1 or 0). reverse = go backward / right instead.
     */
    private boolean move(String name, double forwardDir, double leftDir, double inches, boolean reverse) {
        double sign = reverse ? -1 : 1;
        pinpoint.update();
        double startX = pinpoint.getPosition().getX(DistanceUnit.INCH);   // START: remember where we are
        double startY = pinpoint.getPosition().getY(DistanceUnit.INCH);
        ElapsedTime timer = new ElapsedTime();

        while (true) {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            double traveled = Math.hypot(pose.getX(DistanceUnit.INCH) - startX,
                    pose.getY(DistanceUnit.INCH) - startY);                // CHECK: how far have we gone?
            double remaining = inches - traveled;

            if (remaining <= toleranceIn) return true;                     // STOP: we made it
            if (cancelled() || timer.seconds() > moveTimeoutS) return false;

            double power = remaining < slowZoneIn ? slowPower : fastPower; // slow down near the end
            double turn = headingCorrection(0);                            // stay facing the corner's angle
            setWheels(sign * forwardDir * power, sign * leftDir * power, turn);   // DRIVE

            flywheel.spinTo(shooterSetting * TuningConfig.shooterMaxTps);  // spin up while we drive
            show(name, String.format("%.1f of %.1f in", traveled, inches));
        }
    }

    /** Turn in place until we face `targetDeg`. */
    private boolean turnTo(double targetDeg) {
        ElapsedTime timer = new ElapsedTime();
        while (true) {
            pinpoint.update();
            double error = AngleUnit.normalizeDegrees(targetDeg - heading());
            if (Math.abs(error) <= toleranceDeg) return true;
            if (cancelled() || timer.seconds() > moveTimeoutS) return false;

            setWheels(0, 0, headingCorrection(targetDeg));
            flywheel.spinTo(shooterSetting * TuningConfig.shooterMaxTps);
            show("Turning", String.format("%.1f deg to go", error));
        }
    }

    // =====================================================================
    //  Step 3: shoot. Returns false if cancelled.
    // =====================================================================
    public boolean shoot() {
        double target = shooterSetting * TuningConfig.shooterMaxTps;
        ElapsedTime timer = new ElapsedTime();

        // Wait for the flywheel to reach speed (but not forever).
        while (!flywheel.isReady() && timer.seconds() < TuningConfig.autoSpoolTimeoutS) {
            if (cancelled()) return stopAll();
            flywheel.spinTo(target);
            show("Spooling", String.format("%.0f of %.0f tps", flywheel.getVelocity1(), target));
        }

        // Feed the balls.
        timer.reset();
        while (timer.milliseconds() < feedMs) {
            if (cancelled()) return stopAll();
            flywheel.spinTo(target);
            intake.setPower(TuningConfig.intakePower);
            windmill.setPower(TuningConfig.windmillPower);
            show("Shooting", String.format("%.1f s", timer.seconds()));
        }
        stopAll();
        return true;
    }

    // =====================================================================
    //  Helpers
    // =====================================================================
    /** Autonomous sets this to false so a bumped controller can't cancel the run. */
    public boolean driverCanCancel = true;

    /** Stop if the OpMode ended, or (TeleOp) driver 1 grabbed a stick or pressed B. */
    private boolean cancelled() {
        if (!opMode.opModeIsActive()) return true;
        if (!driverCanCancel) return false;
        double sticks = Math.abs(opMode.gamepad1.left_stick_x) + Math.abs(opMode.gamepad1.left_stick_y)
                + Math.abs(opMode.gamepad1.right_stick_x);
        return sticks > 0.3 || opMode.gamepad1.b;
    }

    /** Turn power that points us at targetDeg. This sign matches DriveMode's snap turn (snapYawSign = 1). */
    private double headingCorrection(double targetDeg) {
        double error = AngleUnit.normalizeDegrees(targetDeg - heading());
        double turn = headingKp * error;
        return Math.max(-maxTurnPower, Math.min(maxTurnPower, turn));
    }

    private double heading() {
        return pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
    }

    /** forward > 0 = forward, left > 0 = strafe left, turn > 0 = same direction as DriveMode's snap turn. */
    private void setWheels(double forward, double left, double turn) {
        double fl = forward + left + turn;
        double bl = forward - left + turn;
        double fr = forward - left - turn;
        double br = forward + left - turn;
        double biggest = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(bl)), Math.max(Math.abs(fr), Math.abs(br))));
        frontLeft.setPower(fl / biggest);
        backLeft.setPower(bl / biggest);
        frontRight.setPower(fr / biggest);
        backRight.setPower(br / biggest);
    }

    private void stopWheels() {
        setWheels(0, 0, 0);
    }

    private boolean stopAll() {
        stopWheels();
        flywheel.stop();
        intake.setPower(0);
        windmill.setPower(0);
        return false;
    }

    /** False until someone measures the spot and types it in (all three are 0 by default). */
    public boolean hasShootingSpot() {
        return shotForwardIn != 0 || shotLeftIn != 0 || shotHeadingDeg != 0;
    }

    public void stop() {
        stopAll();
    }

    /** Where are we, measured from the corner? Use these to measure the shooting spot. */
    public void addPositionTelemetry() {
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        // This Pinpoint reads forward and left as negative, so flip them to read like a tape measure.
        opMode.telemetry.addData("Forward from corner (in)", "%.1f", -pose.getX(DistanceUnit.INCH));
        opMode.telemetry.addData("Left from corner (in)", "%.1f", -pose.getY(DistanceUnit.INCH));
        opMode.telemetry.addData("Heading (deg)", "%.1f", heading());
    }

    private void show(String step, String detail) {
        opMode.telemetry.addData("Corner shot", step);
        opMode.telemetry.addData("Progress", detail);
        opMode.telemetry.addData("Cancel", "move a stick or press B");
        addPositionTelemetry();
        opMode.telemetry.update();
    }
}
