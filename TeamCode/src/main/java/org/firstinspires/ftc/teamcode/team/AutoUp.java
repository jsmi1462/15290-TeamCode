package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Autonomous(name = "Autonomous Up", group = "Sensor")
public class AutoUp extends LinearOpMode {
    private static final double DRIVE_POWER = 0.3;
    private static final double TARGET_X_IN = 82.0;   // forward distance
    private static final double TARGET_Y_IN = 50;   // strafe distance (y = inches left)
    private static final double FIRST_STRAFE_Y_IN = TARGET_Y_IN - 20;
    private static final double TOLERANCE_IN = 0.5;
    private static final double TIMEOUT_S = 28.0;

    private enum State {FIRST_FORWARD, SECOND_FORWARD, FIRST_STRAFE, SECOND_STRAFE, DONE}

    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;
    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

    @Override

    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");

        // Set once. Never flip directions mid-run; use power signs to strafe.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        configurePinpoint();

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();
        runtime.reset();
        AutoUp.State state = AutoUp.State.FIRST_FORWARD;

        while (opModeIsActive() && state != AutoUp.State.DONE) {
            pinpoint.update(); // required every loop or position never changes
            Pose2D pose = pinpoint.getPosition();
            double x = pose.getX(DistanceUnit.INCH);
            double y = -pose.getY(DistanceUnit.INCH); // Pinpoint +Y is right on this robot, so y = inches left

            if (runtime.seconds() > TIMEOUT_S) {
                state = AutoUp.State.DONE;
            }

            switch (state) {
                case FIRST_FORWARD:
                    if (x < (TARGET_X_IN - 55) - TOLERANCE_IN) {
                        drive(DRIVE_POWER, 0);
                    } else {
                        stopDrive();
                        state = AutoUp.State.FIRST_STRAFE;
                    }
                    break;

                case FIRST_STRAFE:
                    if (Math.abs(FIRST_STRAFE_Y_IN - y) > TOLERANCE_IN) {
                        strafeToward(FIRST_STRAFE_Y_IN, y);
                    } else {
                        stopDrive();
                        state = State.SECOND_FORWARD;
                    }
                    break;
                case SECOND_FORWARD:
                    if (x < TARGET_X_IN - TOLERANCE_IN) {
                        drive(DRIVE_POWER, 0);
                    } else {
                        stopDrive();
                        state = AutoUp.State.SECOND_STRAFE;
                    }
                    break;

                case SECOND_STRAFE:
                    if (Math.abs(TARGET_Y_IN - y) > TOLERANCE_IN) {
                        strafeToward(TARGET_Y_IN, y);
                    } else {
                        stopDrive();
                        state = State.DONE;
                    }
                    break;
                case DONE:
                    break;
            }

            addLimelightTelemetry();
            telemetry.addData("State", state);
            telemetry.addData("X (in)", "%.2f", x);
            telemetry.addData("Y (in)", "%.2f", y);
            telemetry.addData("Heading (deg)", "%.1f", pose.getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
        stopDrive();
        limelight.stop();
    }

    /**
     * forward > 0 drives forward, strafe > 0 strafes left (mecanum).
     */
    private void drive(double forward, double strafe) {
        frontLeftDrive.setPower(forward - strafe);
        backLeftDrive.setPower(forward + strafe);
        frontRightDrive.setPower(forward + strafe);
        backRightDrive.setPower(forward - strafe);
    }

    /**
     * Strafes toward targetY; y is inches left (Pinpoint Y negated), so a positive target goes left and a negative one right.
     * On this robot a positive strafe power physically moves right, so the sign is negated.
     */
    private void strafeToward(double targetY, double currentY) {
        drive(0, -Math.signum(targetY - currentY) * DRIVE_POWER);
    }

    private void stopDrive() {
        drive(0, 0);
    }

    private void addLimelightTelemetry() {
        LLStatus status = limelight.getStatus();
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(), (int) status.getFps());

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            telemetry.addData("tx", "%.2f", result.getTx());
            telemetry.addData("ty", "%.2f", result.getTy());
            telemetry.addData("Botpose", botpose.toString());
        } else {
            telemetry.addData("Limelight", "No data");
        }
    }

    private void configurePinpoint() {
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
        sleep(300); // let the IMU finish calibrating before reading
    }
}