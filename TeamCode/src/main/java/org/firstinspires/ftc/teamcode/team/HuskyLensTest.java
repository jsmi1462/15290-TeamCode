package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Prints everything the HuskyLens sees. Use it to check wiring and to learn what the numbers mean:
 * hold a tag in front of the camera and move it left/right (x changes) and closer/farther (width changes).
 * The algorithm comes from TuningConfig.huskyLensAlgorithm.
 */
@TeleOp(name = "HuskyLens Test", group = "Test")
public class HuskyLensTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        HuskyLensHelper huskyLens = new HuskyLensHelper(hardwareMap);

        if (!huskyLens.isConnected()) {
            telemetry.addData(">>", "No \"huskylens\" in the Robot Configuration");
        } else if (!huskyLens.isResponding()) {
            telemetry.addData(">>", "HuskyLens not answering. Check the I2C cable and Protocol Type = I2C.");
        } else {
            telemetry.addData(">>", "HuskyLens OK. Press START.");
        }
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Algorithm", TuningConfig.huskyLensAlgorithm);
            telemetry.addData("Block count", blocks.length);
            for (HuskyLens.Block b : blocks) {
                telemetry.addData("id " + b.id, "x %d  y %d  w %d  h %d", b.x, b.y, b.width, b.height);
            }
            telemetry.update();
        }
    }
}
