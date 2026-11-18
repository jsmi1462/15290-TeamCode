package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Timer;
import java.util.concurrent.TimeUnit;

/**
 * Shoot, then drive the path below. Each line runs to completion before the next one starts.
 * Distances are inches measured from where that move begins. How a move works: see BaseAuto.
 */
@Autonomous(name = "Autonomous Up", group = "Sensor")
public class AutoUp extends BaseAuto {
    private GoBildaPinpointDriver pinpoint;
    Timer timer = new Timer();
    int num = 3000;
    long longNum = num;

    protected void LeftTurn45() {
        // Target: Turn left 45 degrees from current heading
        pinpoint.update();
        double startHeading = pinpoint.getHeading(AngleUnit.DEGREES);
        double targetHeading = startHeading + 45.0;

        // Simple proportional gain (tune kP to prevent overshooting)
        double kP = 0.025;
        double tolerance = 1.0; // Stop within 1 degree of target

        while (opModeIsActive()) {
            pinpoint.update();
            double currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);

            // Calculate shortest angular error (-180 to 180)
            double error = AngleUnit.normalizeDegrees(targetHeading - currentHeading);

            if (Math.abs(error) <= tolerance) {
                break;
            }

            // Positive error = turn left (counterclockwise)
            double turnPower = Range.clip(error * kP, -0.6, 0.6);

            // Minimum power floor to overcome static friction near the target
            if (Math.abs(turnPower) < 0.12) {
                turnPower = Math.signum(error * .12);
            }

            // Standard Mecanum / Tank counterclockwise turn
            frontLeftDrive.setPower(-turnPower);
            backLeftDrive.setPower(-turnPower);
            frontRightDrive.setPower(turnPower);
            backRightDrive.setPower(turnPower);
        }
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);

    }


    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        waitForStart();
        pinpoint = hardwareMap.get(GoBildaPinpointDriver .class, "pinpoint");
        flywheel.spinTo(0.5 * TuningConfig.shooterMaxTps);
        TimeUnit.SECONDS.sleep(3);
        startAuto();   // remember the heading right before we start driving

        forward(3);
        LeftTurn45();
        windmill.setPower(1);
        intake.setPower(1);
        TimeUnit.SECONDS.sleep(3);
        windmill.setPower(0);
        intake.setPower(0);
        LeftTurn45();
        forward(86);
        strafeLeft(3);

        //I think i just plan to move by distance and change those values, it makes more sence to me to just use distance
        //forward is forward(number of inches);
        //back is back(number of inches);
        //strafeLeft is strafeLeft(number of inches);
        //strafeRight is strafeRight(number of inches);
        //windmill is windmill.setPower(0);

        endAuto();
    }
}
