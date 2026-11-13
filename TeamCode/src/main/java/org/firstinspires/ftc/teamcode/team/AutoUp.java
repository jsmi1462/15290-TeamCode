package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Shoot, then drive the path below. Each line runs to completion before the next one starts.
 * Distances are inches measured from where that move begins. How a move works: see BaseAuto.
 */
@Autonomous(name = "Autonomous Up", group = "Sensor")
public class AutoUp extends BaseAuto {

    @Override
    public void runOpMode() {
        initRobot();
        waitForStart();
        startAuto();   // remember the heading right before we start driving

        forward(27);
        strafeLeft(39);
        forward(59);

        endAuto();
    }
}
