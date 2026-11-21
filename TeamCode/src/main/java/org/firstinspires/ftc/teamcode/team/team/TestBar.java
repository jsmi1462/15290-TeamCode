package org.firstinspires.ftc.teamcode.team.team;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TestBar")
public class TestBar extends LinearOpMode {

    @Override
    public void runOpMode() {

        Servo barServo = hardwareMap.get(Servo.class, "barServo");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.dpad_up) {
                barServo.setPosition(0.0);
            }

            if (gamepad2.dpad_right) {
                barServo.setPosition(0.5);
            }

            if (gamepad2.dpad_left) {
                barServo.setPosition(1.0);
            }
        }
    }
}
