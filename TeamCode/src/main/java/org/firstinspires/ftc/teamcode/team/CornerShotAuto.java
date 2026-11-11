package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * AUTONOMOUS using CornerShot. Place the robot backed into the corner before pressing INIT.
 * The same three steps as the TeleOp button, with no button: the start of auto is the button press.
 */
@Autonomous(name = "Corner Shot Auto", group = "Test")
public class CornerShotAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        CornerShot cornerShot = new CornerShot(this);
        cornerShot.driverCanCancel = false;

        // While waiting for START, show whether the robot is placed correctly.
        while (opModeInInit()) {
            telemetry.addData("Corner", cornerShot.inCorner() ? "IN CORNER - ready" : "NOT in corner - fix placement");
            telemetry.update();
        }

        if (!cornerShot.hasShootingSpot()) return;   // nobody has measured the spot yet: do nothing
        cornerShot.resetPose();
        if (cornerShot.driveToShootingSpot()) {
            cornerShot.shoot();
        }
        cornerShot.stop();
    }
}
