package org.firstinspires.ftc.teamcode.team;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * An autonomous written as a state machine. State 0 is a finished example. The rest is yours.
 *
 * How to read this file (new to Java? start here):
 *   - Anything after // or between slash-star blocks like this one is a comment. The robot ignores it.
 *   - Every instruction ends with a semicolon ;   Curly braces { } group instructions together.
 *   - Each "case 0:", "case 1:" is one state. The robot runs only the state it is currently in.
 *     "break;" ends a state. Without it the robot falls into the next one.
 *   - A name shown in red means Android Studio does not know it yet. Click it, press Alt+Enter
 *     (Option+Enter on Mac), and choose Import class.
 *
 * Questions to answer before you write state 1:
 *   - What has to be true before the robot is allowed to move on from state 0?
 *   - Why does follower.update() sit at the top of the loop, outside the switch?
 *   - What would happen if we called follower.follow(...) every loop instead of once?
 *     (That is the job of the `entering` flag.)
 *   - Which states move the robot, and which ones just wait for a mechanism?
 */
@Autonomous(name = "Template Auto", group = "Team")
public class TemplateAuto extends LinearOpMode {

    private Follower follower;
    private LimelightHelper limelight;
    private final ElapsedTime timer = new ElapsedTime();

    private int state = 0;
    private boolean entering = true;

    private void next() {
        state++;
        entering = true;
        timer.reset();
    }

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        limelight = new LimelightHelper(hardwareMap);

        follower.setPose(Poses.START);

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            follower.update();

            switch (state) {
                case 0: // EXAMPLE: drive from START to SCORE
                    if (entering) {
                        follower.follow(line(Poses.START, Poses.SCORE).linear(Poses.START, Poses.SCORE));
                        entering = false;
                    }
                    if (!follower.isBusy()) {
                        limelight.relocalize(follower); // fix odometry drift while stopped
                        next();
                    }
                    break;

                case 1:
                    // TODO: your next step
                    break;

                default:
                    break;
            }

            telemetry.addData("state", state);
            telemetry.addData("pose", follower.pose());
            telemetry.update();
        }

        limelight.stop();
    }
}
