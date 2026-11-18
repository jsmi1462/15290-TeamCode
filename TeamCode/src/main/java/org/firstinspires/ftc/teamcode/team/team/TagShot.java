
        package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

initRobot();

@TeleOp(name = "Tag Shot", group = "Test")
public class TagShot extends BaseAuto {

    static final int TAG_ID = 0;   // step 2 tells you the real number

    @Override
    public void runOpMode() {
        initRobot();
        waitForStart();
        while (opModeIsActive()) {
            // steps 2 to 6 go here
            telemetry.update();
        }
        endAuto();
    }
}
