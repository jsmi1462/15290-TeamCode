package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Drive the path below. Each line runs to completion before the next one starts.
 * Distances are inches measured from where that move begins. How a move works: see BaseAuto.
 */
@Autonomous(name = "Autonomous Down", group = "Sensor")
public class AutoDown extends BaseAuto {

    @Override
    public void runOpMode() {
        initRobot();
        waitForStart();
        startAuto();

        forward(13);
        strafeRight(47);

        endAuto();
    }
}
