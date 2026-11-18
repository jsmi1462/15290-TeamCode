package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class LimeLightTest extends OpMode {
    private Limelight3A limelight = null;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

    }

}
