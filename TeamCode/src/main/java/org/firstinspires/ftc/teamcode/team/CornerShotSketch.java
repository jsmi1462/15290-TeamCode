package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * CORNER SHOT: YOUR VERSION.  Fill in every spot marked  TODO.
 *
 * THE PLAN (your pseudocode, cleaned up)
 *   1. CORNER?  Both distance sensors read about 2 inches.
 *   2. RESET    Driver presses X: tell the Pinpoint "you are at 0, 0, 0".
 *   3. DRIVE    Go forward until we have traveled far enough.  (start, drive, check, stop)
 *   4. TURN     Turn until we face the shooting angle.
 *   5. SHOOT    Flywheel, intake and windmill on.
 *
 * QUESTIONS TO ANSWER BEFORE YOU CODE
 *   a. A distance sensor reads 2.03, then 1.98, then 2.01. Why will   back == 2   almost never be true?
 *      What should we ask instead?  (Hint: Math.abs)
 *   b. Your pseudocode put the TURN loop INSIDE the DRIVE loop. What would the robot do?
 *   c. Your pseudocode set the motors to 0 whenever we are in the corner. In TeleOp, how would the
 *      driver ever drive back OUT of the corner?
 *   d. Why does pinpoint.update() have to be inside every loop?
 *   e. On this robot, driving forward makes the Pinpoint x go NEGATIVE. Why does measuring
 *      "distance traveled" with Math.hypot mean we don't care?
 *
 * When it works, delete the @Disabled line so it shows up on the Driver Station.
 * (The finished version is CornerShot.java. Don't peek until you've tried!)
 */
@Disabled
@TeleOp(name = "Corner Shot Sketch", group = "Test")
public class CornerShotSketch extends LinearOpMode {

    // TODO: measure these. Back into the corner, reset, drive by hand to the best spot, read the screen.
    static final double SHOT_FORWARD_IN = 0;
    static final double SHOT_HEADING_DEG = 0;
    static final double CORNER_IN = 2.0;      // what both sensors read in the corner
    static final double CORNER_TOL_IN = 0.75; // how close to CORNER_IN counts

    DcMotor frontLeft, backLeft, frontRight, backRight, intake, windmill;
    DistanceSensor distBack, distRight;
    GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "intake");
        windmill = hardwareMap.get(DcMotor.class, "windmill");
        distBack = hardwareMap.get(DistanceSensor.class, "distBack");
        distRight = hardwareMap.get(DistanceSensor.class, "distRight");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        // (Shooter left out to keep this short. Add it in step 5 if you want.)

        waitForStart();

        while (opModeIsActive()) {
            pinpoint.update();

            // ---------- 1. CORNER? ----------
            double back = distBack.getDistance(DistanceUnit.INCH);
            double right = distRight.getDistance(DistanceUnit.INCH);
            boolean inCorner = false;   // TODO: true when BOTH back and right are within CORNER_TOL_IN of CORNER_IN

            if (inCorner && gamepad1.x) {

                // ---------- 2. RESET ----------
                // TODO: set the Pinpoint to x=0, y=0, heading=0.
                //       Look for pinpoint.setPosition(new Pose2D(...)) or pinpoint.resetPosAndIMU().

                // ---------- 3. DRIVE (start, drive, check, stop) ----------
                double startX = 0;      // TODO: START: read where we are right now
                double startY = 0;      // TODO
                double traveled = 0;
                while (opModeIsActive() && traveled < SHOT_FORWARD_IN) {
                    pinpoint.update();
                    // TODO: CHECK: traveled = Math.hypot(x now - startX, y now - startY)
                    // TODO: DRIVE: all four wheels forward. Why might 0.8 overshoot? What could you do near the end?
                    break; // TODO: delete this line once the loop can finish on its own
                }
                // TODO: STOP: all four wheels to 0

                // ---------- 4. TURN ----------
                // Error = where we want to face minus where we face now, wrapped to -180..180.
                double error = AngleUnit.normalizeDegrees(SHOT_HEADING_DEG - heading());
                while (opModeIsActive() && Math.abs(error) > 1.5) {
                    pinpoint.update();
                    error = AngleUnit.normalizeDegrees(SHOT_HEADING_DEG - heading());
                    double turn = 0.03 * error;   // farther away = turn harder
                    // TODO: left wheels get +turn, right wheels get -turn
                    break; // TODO: delete this line once the loop can finish on its own
                }
                // TODO: STOP the wheels

                // ---------- 5. SHOOT ----------
                // TODO: intake and windmill to full power. How long should they run? How do they stop?
            }

            // TODO: normal driving with the sticks goes here (copy it from DriveMode)

            telemetry.addData("Back / right (in)", "%.1f / %.1f", back, right);
            telemetry.addData("In corner", inCorner);
            telemetry.addData("x / y (in)", "%.1f / %.1f",
                    pinpoint.getPosition().getX(DistanceUnit.INCH), pinpoint.getPosition().getY(DistanceUnit.INCH));
            telemetry.addData("Heading (deg)", "%.1f", heading());
            telemetry.update();
        }
    }

    double heading() {
        return pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
    }
}
