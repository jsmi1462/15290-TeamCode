package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Stationary shooter tuning. No driving. Open FTC Dashboard, change TuningConfig values, and watch the graphs.
 * Gamepad 1: D-pad up/down = shooter power +/- 0.05, A = zero the Pinpoint here, B = stop flywheels.
 * Gamepad 2: A = intake, left bumper = windmill, B = stop flywheels.
 * Push the robot to a spot to read its exact position and angle (same numbers BaseAuto and CornerShot use).
 */
@TeleOp(name = "Shooter Tuning", group = "Test")
public class ShooterTuning extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Sends telemetry to both the Driver Station and the dashboard (graphs numeric values).
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        DcMotorEx shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotor windmill = hardwareMap.get(DcMotor.class, "windmill");
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Same directions as DriveMode.
        shooter1.setDirection(DcMotor.Direction.FORWARD);
        shooter2.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        windmill.setDirection(DcMotor.Direction.FORWARD);
        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ShooterFlywheel flywheel = new ShooterFlywheel(shooter1, shooter2);
        flywheel.init();

        // Pinpoint: same setup as BaseAuto and CornerShot, so the numbers here can be copied straight into them.
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
        sleep(300); // let the IMU finish calibrating before reading
        boolean aWasPressed = false;

        boolean upWasPressed = false;
        boolean downWasPressed = false;
        int appliedPipeline = -1;
        int appliedPollHz = -1;
        limelight.start();

        telemetry.addData("Status", "Ready. Open the dashboard and press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (TuningConfig.limelightPipeline != appliedPipeline) {
                appliedPipeline = TuningConfig.limelightPipeline;
                limelight.pipelineSwitch(appliedPipeline);
            }
            if (TuningConfig.limelightPollHz != appliedPollHz) {
                appliedPollHz = TuningConfig.limelightPollHz;
                limelight.setPollRateHz(Math.max(1, Math.min(250, appliedPollHz)));
            }

            // D-pad up / down on gamepad 1 changes the power by 0.05 (works even without FTC Dashboard).
            if (gamepad1.dpad_up && !upWasPressed) {
                TuningConfig.shooterPower = Math.min(1.0, Math.round((TuningConfig.shooterPower + 0.05) * 100) / 100.0);
            }
            if (gamepad1.dpad_down && !downWasPressed) {
                TuningConfig.shooterPower = Math.max(0.0, Math.round((TuningConfig.shooterPower - 0.05) * 100) / 100.0);
            }
            upWasPressed = gamepad1.dpad_up;
            downWasPressed = gamepad1.dpad_down;

            boolean stop = gamepad1.b || gamepad2.b;
            double power = stop ? 0.0 : TuningConfig.shooterPower;
            // Same closed-loop control as DriveMode: shooterPower 1.0 = shooterMaxTps.
            flywheel.spinTo(power * TuningConfig.shooterMaxTps);
            intake.setPower(gamepad2.a ? TuningConfig.intakePower : 0.0);
            windmill.setPower(gamepad2.left_bumper ? TuningConfig.windmillPower : 0.0);

            double v1 = shooter1.getVelocity();
            double v2 = shooter2.getVelocity();
            // ---------- Position and angle ----------
            // A (gamepad 1) = zero here. Push the robot to a spot and read the numbers.
            pinpoint.update();
            if (gamepad1.a && !aWasPressed) {
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
                pinpoint.update();
            }
            aWasPressed = gamepad1.a;
            Pose2D pose = pinpoint.getPosition();
            // This Pinpoint reads forward and left as NEGATIVE, so the "from zero" lines flip them
            // to read like a tape measure. Those are the numbers CornerShot wants.
            telemetry.addData("Forward from zero (in)", "%.2f", -pose.getX(DistanceUnit.INCH));
            telemetry.addData("Left from zero (in)", "%.2f", -pose.getY(DistanceUnit.INCH));
            telemetry.addData("Angle (deg)", "%.2f", pose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Raw Pinpoint x / y (in)", "%.2f / %.2f",
                    pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH));
            telemetry.addData("Pinpoint", "%s   (A = zero here)", pinpoint.getDeviceStatus());

            // These three lines show what the ROBOT received. If they don't match the dashboard, the edit wasn't saved.
            telemetry.addData("Shooter power", power);
            telemetry.addData("Speed control", TuningConfig.useVelocityControl ? "ON (encoders)" : "OFF (plain power)");
            telemetry.addData("shooterMaxTps", TuningConfig.shooterMaxTps);
            telemetry.addData("Shooter1 velocity (tps)", v1);
            telemetry.addData("Shooter2 velocity (tps)", v2);
            telemetry.addData("Shooter1 position (ticks)", shooter1.getCurrentPosition());
            telemetry.addData("Shooter2 position (ticks)", shooter2.getCurrentPosition());
            telemetry.addData("Velocity gap (tps)", Math.abs(v1 - v2));
            telemetry.addData("Target (tps)", flywheel.getTarget());
            telemetry.addData("Shooter ready", flywheel.isReady());

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                telemetry.addData("LL tx", result.getTx());
                telemetry.addData("LL ty", result.getTy());
                telemetry.addData("LL ta", result.getTa());
            } else {
                telemetry.addData("Limelight", "No target");
            }
            telemetry.update();
        }

        flywheel.stop();
        limelight.stop();
    }
}
