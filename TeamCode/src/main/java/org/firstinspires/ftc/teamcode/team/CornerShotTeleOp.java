package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * TEST TELEOP for CornerShot. Drive normally. Back into the corner. When telemetry says
 * "IN CORNER - press X", press X (driver 1) and the robot resets, drives to the shooting spot, and shoots.
 * Move a stick or press B to cancel.
 *
 * Also use this to MEASURE the shooting spot: press X with shotForwardIn/shotLeftIn at 0 (it only resets),
 * drive by hand to the best spot, and copy the "from corner" numbers into CornerShot.
 */
@TeleOp(name = "Corner Shot Test", group = "Test")
public class CornerShotTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        CornerShot cornerShot = new CornerShot(this);
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
        // (CornerShot already set the motor directions.)

        telemetry.addData("Status", "Ready. Press START.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            boolean inCorner = cornerShot.inCorner();

            if (inCorner && gamepad1.x && !cornerShot.hasShootingSpot()) {
                cornerShot.resetPose();   // measuring mode: reset only, no driving or shooting
                telemetry.addData("Last corner shot", "reset only (no shooting spot set yet)");
            } else if (inCorner && gamepad1.x) {
                cornerShot.resetPose();
                boolean ok = cornerShot.driveToShootingSpot() && cornerShot.shoot();
                cornerShot.stop();
                telemetry.addData("Last corner shot", ok ? "done" : "CANCELLED");
            }

            // Normal driving, same as DriveMode.
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;
            frontLeft.setPower(forward + strafe + turn);
            frontRight.setPower(forward - strafe - turn);
            backLeft.setPower(forward - strafe + turn);
            backRight.setPower(forward + strafe - turn);

            telemetry.addData("Corner", inCorner ? "IN CORNER - press X" : "not in corner");
            cornerShot.addPositionTelemetry();
            telemetry.update();
        }
        cornerShot.stop();
    }
}
