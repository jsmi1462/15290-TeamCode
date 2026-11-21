package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.TimeUnit;

/**
 * Shoot, then drive the path below. Each line runs to completion before the next one starts.
 * Distances are inches measured from where that move begins. How a move works: see BaseAuto.
 */
@Autonomous(name = "Autonomous Up", group = "Sensor")
public class AutoUp extends BaseAuto {

    protected void LeftTurn(double turnValue) {
        // Target: Turn left `turnValue` degrees from current heading
        pinpoint.update();
        double startHeading = pinpoint.getHeading(AngleUnit.DEGREES);
        double targetHeading = startHeading + turnValue;

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
                turnPower = Math.signum(error) * 0.12;
            }

            // Standard Mecanum / Tank counterclockwise turn
            frontLeftDrive.setPower(turnPower);
            backLeftDrive.setPower(turnPower);
            frontRightDrive.setPower(-turnPower);
            backRightDrive.setPower(-turnPower);
        }
        stopDrive();

        // Save the new heading so driveInches maintains this new direction
        updateStartHeading();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        waitForStart();
        startAuto();   // remember the heading right before we start driving

        flywheel.spinTo(0.31 * TuningConfig.shooterMaxTps);
        TimeUnit.SECONDS.sleep(3);

        forward(3);
        strafeLeft(3);
        LeftTurn(33);
        windmill.setPower(1);
        intake.setPower(1);
        TimeUnit.SECONDS.sleep(3);
        windmill.setPower(0);
        intake.setPower(0);
        LeftTurn(50);
        forward(80);
        strafeLeft(3);

        endAuto();
    }
}
