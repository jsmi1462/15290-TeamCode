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

@Autonomous(name = "Autonomous Down", group = "Sensor")
public class AutoDown extends LinearOpMode {
    private static final double DRIVE_POWER = 0.3;
    private static final double TARGET_X_IN = 13.0;   // forward distance
    private static final double TARGET_Y_IN = 47.0;   // strafe distance (raw Pinpoint y, + = right)

    // Heading hold. Flip HEADING_GAIN's sign if the robot spins faster instead of straightening out.
    private static final double HEADING_GAIN = 0.5;           // turn power per radian of heading error
    private static final double MAX_HEADING_ERROR_RAD = 0.8;  // ~45 deg: something is wrong, stop instead of spinning
    private static final double TOLERANCE_IN = 0.5;
    private static final double TIMEOUT_S = 28.0;

    private enum State { FIRST_FORWARD, FIRST_STRAFE, DONE }

    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;
    private double headingErrorRad;   // current heading minus the heading at the start of the run
    private double startHeadingRad;
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
        pinpoint.update();
        startHeadingRad = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
        AutoDown.State state = AutoDown.State.FIRST_FORWARD;

        while (opModeIsActive() && state != AutoDown.State.DONE) {
            pinpoint.update(); // required every loop or position never changes
            Pose2D pose = pinpoint.getPosition();
            headingErrorRad = pose.getHeading(AngleUnit.RADIANS) - startHeadingRad;
            double x = pose.getX(DistanceUnit.INCH);
            double y = pose.getY(DistanceUnit.INCH);  // raw Pinpoint: right reads positive, left negative

            if (runtime.seconds() > TIMEOUT_S || Math.abs(headingErrorRad) > MAX_HEADING_ERROR_RAD) {
                state = AutoDown.State.DONE;
            }

            switch (state) {
                case FIRST_FORWARD:
                    if (x < TARGET_X_IN - TOLERANCE_IN ) {
                        drive(DRIVE_POWER, 0);
                    } else {
                        stopDrive();
                        state = AutoDown.State.FIRST_STRAFE;
                    }
                    break;

                case FIRST_STRAFE:
                    if (y < TARGET_Y_IN - TOLERANCE_IN) {
                        drive(0, -DRIVE_POWER); // strafe right
                    } else {
                        stopDrive();
                        state = AutoDown.State.DONE;
                    }
                    break;
                case DONE:
                    break;
            }

            addLimelightTelemetry();
            telemetry.addData("State", state);
            telemetry.addData("X (in)", "%.2f", x);
            telemetry.addData("Y (in)", "%.2f", y);
            telemetry.addData("Heading error (deg)", "%.1f", Math.toDegrees(headingErrorRad));
            telemetry.update();
        }
        stopDrive();
        limelight.stop();
    }
    /** forward > 0 drives forward, strafe > 0 strafes left (mecanum), and the turn term holds the starting heading. */
    private void drive(double forward, double strafe) {
        double turn = headingErrorRad * HEADING_GAIN;
        frontLeftDrive.setPower(forward + strafe - turn);
        backLeftDrive.setPower(forward - strafe - turn);
        frontRightDrive.setPower(forward - strafe + turn);
        backRightDrive.setPower(forward + strafe + turn);
    }

    private void stopDrive() {
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
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