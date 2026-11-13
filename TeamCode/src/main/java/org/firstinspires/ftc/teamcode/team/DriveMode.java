/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/*
 * DRIVE MODE (the program the drivers run during teleop)
 *
 * CONTROLS
 *   Driver 1:  left stick = drive and strafe, right stick = turn
 *              left trigger / right trigger = turn the robot to a set angle
 *              Y = shooter faster, X = shooter slower
 *              B = emergency stop for the shooter
 *   Driver 2:  A (hold) = intake, left bumper (hold) = windmill
 *              B = emergency stop for the shooter
 *
 * WANT TO CHANGE A NUMBER (power, angle, distance)?
 *   Open TuningConfig.java. Every number has a comment that says what it does.
 *   You can also change them live in FTC Dashboard while the robot is on.
 */
@TeleOp(name="DriveMode", group="Linear OpMode")
public class DriveMode extends LinearOpMode {

    // ---- The robot's parts. The names in quotes below must match the Robot Configuration. ----
    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private ShooterFlywheel flywheel;   // runs both shooter motors at a steady speed using their encoders
    private DcMotor intake;
    private DcMotor windmill;
    private GoBildaPinpointDriver pinpoint;  // tells us where the robot is and which way it faces
    private Limelight3A limelight;
    private DistanceSensor distBack;    // back-right corner of the robot, points at the back wall
    private DistanceSensor distRight;   // back-right corner of the robot, points at the right wall

    @Override
    public void runOpMode() {

        // ================= STEP 1: connect to the robot's parts =================
        CornerShot cornerShot = new CornerShot(this);
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        intake = hardwareMap.get(DcMotor.class, "intake");
        windmill = hardwareMap.get(DcMotor.class, "windmill");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // tryGet = if a sensor is missing, skip the corner reset instead of crashing.
        distBack = hardwareMap.tryGet(DistanceSensor.class, "distBack");
        distRight = hardwareMap.tryGet(DistanceSensor.class, "distRight");

        // ================= STEP 2: set which way each motor spins =================
        // If the robot drives backward when you push the stick forward, flip FORWARD and REVERSE here.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        shooter1.setDirection(DcMotor.Direction.FORWARD);
        shooter2.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        windmill.setDirection(DcMotor.Direction.FORWARD);

        flywheel = new ShooterFlywheel(shooter1, shooter2);
        flywheel.init();

        telemetry.addData("Status", "Initialized. Press START.");
        telemetry.update();
        waitForStart();

        // ================= STEP 3: things we remember between loops =================
        double shooterPower = 0.2;          // 0.2 = 20% speed. Driver 1 raises or lowers it.
        boolean yWasPressed = false;        // lets us count one tap as one step, not a held button
        boolean xWasPressed = false;
        ElapsedTime cornerTimer = new ElapsedTime();   // how long we have been sitting in the corner
        boolean cornerDone = false;         // true once the position was reset for this visit
        int pipelineInUse = -1;             // the Limelight pipeline we last switched to
        int pollRateInUse = -1;             // the Limelight update rate we last set
        limelight.start();

        // ================= STEP 4: the loop. Everything below repeats until STOP. =================
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
            pinpoint.update();   // get a fresh position and angle
            double heading = pinpoint.getPosition().getHeading(AngleUnit.DEGREES);

            DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
            // (CornerShot already set the motor directions.)

            telemetry.addData("Status", "Ready. Press START.");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {

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

            // ---------- DRIVING ----------
            double forward = -gamepad1.left_stick_y;   // stick forward is negative, so flip it
            double strafe = gamepad1.left_stick_x;     // sideways
            double turn = -gamepad1.right_stick_x;     // flipped because this robot turns the other way

            // ---------- SNAP TURN: hold a trigger and the robot turns to a set angle ----------
            boolean snapping = gamepad1.left_trigger > 0.5 || gamepad1.right_trigger > 0.5;
            double snapAngle = gamepad1.left_trigger > 0.5 ? TuningConfig.snapLeftDeg : TuningConfig.snapRightDeg;
            if (snapping) {
                // How far off are we? (angle we want) minus (angle we have), wrapped to -180..180.
                double angleError = AngleUnit.normalizeDegrees(snapAngle - heading);
                if (Math.abs(angleError) < TuningConfig.snapDoneDeg) {
                    turn = 0;   // close enough, stop turning
                } else {
                    // Turn faster when far away, slower when close. Never faster than the max.
                    turn = TuningConfig.snapYawSign * TuningConfig.snapKp * angleError;
                    turn = Math.max(-TuningConfig.snapMaxPower, Math.min(TuningConfig.snapMaxPower, turn));
                }
            }

            // Mix the three movements into a power for each wheel.
            double frontLeftPower = forward + strafe + turn;
            double frontRightPower = forward - strafe - turn;
            double backLeftPower = forward - strafe + turn;
            double backRightPower = forward + strafe - turn;

            // Motors cannot go above 1.0. If any wheel is over, shrink all four by the same amount.
            double biggest = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
            if (biggest > 1.0) {
                frontLeftPower /= biggest;
                frontRightPower /= biggest;
                backLeftPower /= biggest;
                backRightPower /= biggest;
            }

            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            // ---------- SHOOTER ----------
            // One tap of Y or X (driver 1) changes the speed by 0.1.
            if (gamepad2.y && !yWasPressed) {
                shooterPower = Math.min(1.0, Math.round((shooterPower + 0.1) * 10) / 10.0);
            }
            if (gamepad2.x && !xWasPressed) {
                shooterPower = Math.max(0.2, Math.round((shooterPower - 0.1) * 10) / 10.0);
            }
            yWasPressed = gamepad2.y;
            xWasPressed = gamepad2.x;

            boolean shooterStop = gamepad1.b || gamepad2.b;   // emergency stop
            double shooterSetting = TuningConfig.overrideShooterPower ? TuningConfig.shooterPower : shooterPower;
            // The shooter holds a steady SPEED using its encoders. 1.0 means full speed (shooterMaxTps).
            flywheel.spinTo(shooterStop ? 0.0 : shooterSetting * TuningConfig.shooterMaxTps);

            // ---------- INTAKE and WINDMILL ----------
            intake.setPower(gamepad2.a ? TuningConfig.intakePower : 0.0);
            windmill.setPower(gamepad2.left_bumper ? TuningConfig.windmillPower : 0.0);


            // ---------- LIMELIGHT (settings come from TuningConfig) ----------
            if (TuningConfig.limelightPipeline != pipelineInUse) {
                pipelineInUse = TuningConfig.limelightPipeline;
                limelight.pipelineSwitch(pipelineInUse);
            }
            if (TuningConfig.limelightPollHz != pollRateInUse) {
                pollRateInUse = TuningConfig.limelightPollHz;
                limelight.setPollRateHz(Math.max(1, Math.min(250, pollRateInUse)));
            }
            LLResult camera = limelight.getLatestResult();
            boolean seesTarget = camera != null && camera.isValid();

            // ---------- WHAT THE DRIVERS SEE ----------
            Pose2D position = pinpoint.getPosition();
            telemetry.addData("Shooter", shooterStop ? "E-STOP" : (flywheel.isReady() ? "READY" : "spooling..."));
            telemetry.addData("Shooter setting", "%.1f", shooterSetting);
            telemetry.addData("Shooter speed target / actual", "%.0f / %.0f", flywheel.getTarget(), flywheel.getVelocity1());
            telemetry.addData("Position x / y (in)", "%.1f / %.1f",
                    position.getX(DistanceUnit.INCH), position.getY(DistanceUnit.INCH));
            telemetry.addData("Angle (deg)", "%.1f", heading);
            telemetry.addData("Corner reset", inCorner);
            telemetry.addData("Snap turn", snapping ? "going to " + snapAngle : "off");
            telemetry.addData("Limelight sees target", seesTarget);
            telemetry.addData("Corner", inCorner ? "IN CORNER - press X" : "not in corner");
            cornerShot.addPositionTelemetry();
            telemetry.update();
        }

        flywheel.stop();
    }
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


            telemetry.addData("Corner", inCorner ? "IN CORNER - press X" : "not in corner");
            cornerShot.addPositionTelemetry();
            telemetry.update();
        }
        cornerShot.stop();
    }
}
