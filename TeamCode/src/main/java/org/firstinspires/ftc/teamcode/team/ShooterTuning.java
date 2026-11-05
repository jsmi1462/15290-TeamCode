package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Stationary shooter tuning. No driving. Open FTC Dashboard, change TuningConfig values, and watch the graphs.
 * A (gamepad2) runs the intake, left bumper runs the windmill, B on either gamepad stops the flywheels.
 */
@TeleOp(name = "ShooterTuning", group = "Tuning")
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

            boolean stop = gamepad1.b || gamepad2.b;
            double power = stop ? 0.0 : TuningConfig.shooterPower;
            // Same closed-loop control as DriveMode: shooterPower 1.0 = shooterMaxTps.
            flywheel.spinTo(power * TuningConfig.shooterMaxTps);
            intake.setPower(gamepad2.a ? TuningConfig.intakePower : 0.0);
            windmill.setPower(gamepad2.left_bumper ? TuningConfig.windmillPower : 0.0);

            double v1 = shooter1.getVelocity();
            double v2 = shooter2.getVelocity();
            telemetry.addData("Shooter power", power);
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
