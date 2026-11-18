package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Shared robot code for our autonomous OpModes. AutoUp and AutoDown "extend" this class,
 * so they can call forward(), strafeLeft(), etc. without copying this code.
 *
 * HOW EVERY MOVE WORKS (this is the whole idea, see driveInches below):
 *   1. START: remember where the robot is right now.
 *   2. DRIVE: set the wheel powers.
 *   3. CHECK: every loop, measure how far we have gone from the start spot.
 *   4. STOP:  when distance traveled >= the distance we asked for, stop the wheels.
 *
 * Distance traveled is always a positive number, so you never have to think about whether
 * the Pinpoint counts forward as + or -. Direction comes ONLY from the power sign, and that is
 * decided in exactly one place: forward() / back() / strafeLeft() / strafeRight().
 */
public abstract class BaseAuto extends LinearOpMode {

    // ---- Numbers you are allowed to change ----
    protected static final double DRIVE_POWER = 0.3;          // wheel power while moving (0 to 1)
    protected static final double TOLERANCE_IN = 0.5;         // stop this many inches early (the robot coasts a little)
    protected static final double HEADING_GAIN = 0.5;         // how hard to correct back to straight (verified sign)
    protected static final double MAX_HEADING_ERROR_RAD = 0.8; // ~45 deg: something is wrong, stop instead of spinning
    protected static final double MOVE_TIMEOUT_S = 10.0;      // give up on a move that takes longer (stuck on a wall)

    protected DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    protected GoBildaPinpointDriver pinpoint;
    protected Limelight3A limelight;
    protected ShooterFlywheel flywheel;
    protected DcMotor intake, windmill;

    private double startHeadingRad;   // the heading we try to keep for the whole auto
    private boolean aborted = false;  // once true, every later move is skipped

    // =====================================================================
    //  The four moves. These are the ONLY places that decide direction.
    // =====================================================================
    protected void forward(double inches)     { driveInches("Forward",      DRIVE_POWER,  0,            inches); }
    protected void back(double inches)        { driveInches("Back",         -DRIVE_POWER, 0,            inches); }
    protected void strafeLeft(double inches)  { driveInches("Strafe left",  0,            DRIVE_POWER,  inches); }
    protected void strafeRight(double inches) { driveInches("Strafe right", 0,            -DRIVE_POWER, inches); }

    /** Drive until the robot has traveled `inches` from where this move started, then stop. */
    private void driveInches(String name, double forwardPower, double strafePower, double inches) {
        if (aborted) return;

        // 1. START: remember where we are.
        pinpoint.update();
        double startX = pinpoint.getPosition().getX(DistanceUnit.INCH);
        double startY = pinpoint.getPosition().getY(DistanceUnit.INCH);
        ElapsedTime moveTimer = new ElapsedTime();
        double traveled = 0;

        while (opModeIsActive()) {
            pinpoint.update(); // required every loop or the position never changes
            Pose2D pose = pinpoint.getPosition();
            double headingError = pose.getHeading(AngleUnit.RADIANS) - startHeadingRad;

            // 3. CHECK: how far have we gone? (Pythagorean theorem, always positive)
            double dx = pose.getX(DistanceUnit.INCH) - startX;
            double dy = pose.getY(DistanceUnit.INCH) - startY;
            traveled = Math.hypot(dx, dy);

            // 4. STOP: are we there yet?
            if (traveled >= inches - TOLERANCE_IN) break;

            // Safety stops. If either fires, the rest of the auto is skipped.
            if (Math.abs(headingError) > MAX_HEADING_ERROR_RAD || moveTimer.seconds() > MOVE_TIMEOUT_S) {
                aborted = true;
                break;
            }

            // 2. DRIVE (with a small turn to stay pointed straight).
            setWheels(forwardPower, strafePower, headingError * HEADING_GAIN);

            telemetry.addData("Move", name);
            telemetry.addData("Traveled", "%.1f of %.1f in", traveled, inches);
            telemetry.addData("Heading error (deg)", "%.1f", Math.toDegrees(headingError));
            telemetry.update();
        }
        stopDrive();
        telemetry.addData("Finished", "%s: %.1f of %.1f in%s", name, traveled, inches, aborted ? "  (ABORTED)" : "");
        telemetry.update();
    }

    /** forward > 0 drives forward, strafe > 0 strafes LEFT on this robot, turn holds our heading. */
    private void setWheels(double forward, double strafe, double turn) {
        frontLeftDrive.setPower(forward + strafe - turn);
        backLeftDrive.setPower(forward - strafe - turn);
        frontRightDrive.setPower(forward - strafe + turn);
        backRightDrive.setPower(forward + strafe + turn);
    }

    protected void stopDrive() {
        setWheels(0, 0, 0);
    }

    // =====================================================================
    //  Shooter
    // =====================================================================
    /** Spin the flywheels up, wait until they are at speed, keep them running for TuningConfig.autoShooterMs, then stop. */
    protected void spinUpAndShoot() {
        double target = TuningConfig.autoShooterPower * TuningConfig.shooterMaxTps;
        ElapsedTime spool = new ElapsedTime();
        while (opModeIsActive() && !flywheel.isReady() && spool.seconds() < TuningConfig.autoSpoolTimeoutS) {
            flywheel.spinTo(target);
            telemetry.addData("Spooling", "%4.0f / %4.0f of %4.0f tps",
                    flywheel.getVelocity1(), flywheel.getVelocity2(), flywheel.getTarget());
            telemetry.update();
        }
        // If the flywheel is barely turning, something is wrong (motor direction, cable). Don't waste
        // the whole auto waiting on it: skip the shot and go drive.
        // Math.abs: an encoder wired backward reads negative while spinning; that still counts as spinning.
        boolean spinning = Math.abs(flywheel.getVelocity1()) > 0.5 * target
                && Math.abs(flywheel.getVelocity2()) > 0.5 * target;
        if (!spinning) {
            flywheel.stop();
            telemetry.addData("SHOOTER PROBLEM", "flywheel not spinning (%.0f / %.0f tps). Skipping the shot.",
                    flywheel.getVelocity1(), flywheel.getVelocity2());
            telemetry.update();
            return;
        }
        // Feed the balls: intake and windmill on while the flywheel holds its speed.
        ElapsedTime feed = new ElapsedTime();
        while (opModeIsActive() && feed.milliseconds() < TuningConfig.autoShooterMs) {
            flywheel.spinTo(target);
            intake.setPower(TuningConfig.intakePower);
            windmill.setPower(TuningConfig.windmillPower);
            telemetry.addData("Shooting", "%.1f of %.1f s", feed.seconds(), TuningConfig.autoShooterMs / 1000.0);
            telemetry.addData("Flywheel", "%4.0f / %4.0f of %4.0f tps",
                    flywheel.getVelocity1(), flywheel.getVelocity2(), target);
            telemetry.update();
        }
        intake.setPower(0);
        windmill.setPower(0);
        flywheel.stop();
    }

    // =====================================================================
    //  Setup: call initRobot() before waitForStart(), startAuto() right after it.
    // =====================================================================
    protected void initRobot() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        for (DcMotor m : new DcMotor[]{frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        DcMotorEx shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        DcMotorEx shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter1.setDirection(DcMotor.Direction.FORWARD);
        shooter2.setDirection(DcMotor.Direction.REVERSE);   // must match DriveMode, or the two motors fight and stall
        flywheel = new ShooterFlywheel(shooter1, shooter2);
        flywheel.init();
        intake = hardwareMap.get(DcMotor.class, "intake");
        windmill = hardwareMap.get(DcMotor.class, "windmill");
        intake.setDirection(DcMotor.Direction.FORWARD);     // same as DriveMode
        windmill.setDirection(DcMotor.Direction.FORWARD);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
        sleep(300); // let the IMU finish calibrating before reading

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(TuningConfig.limelightPipeline);
        limelight.setPollRateHz(Math.max(1, Math.min(250, TuningConfig.limelightPollHz)));
        limelight.start();

        telemetry.setMsTransmissionInterval(11);
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
    }

    /** Remember the starting heading so every move can drive straight relative to it. */
    protected void startAuto() {
        pinpoint.update();
        startHeadingRad = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
    }

    protected void endAuto() {
        stopDrive();
        flywheel.stop();
        intake.setPower(0);
        windmill.setPower(0);
        limelight.stop();
    }
}
