

package org.firstinspires.ftc.teamcode.team.team;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.team.BaseAuto;
import org.firstinspires.ftc.teamcode.team.TuningConfig;

@TeleOp(name = "Tag Shot", group = "Test")
public class TagShot extends BaseAuto {

    // =====================================
    // APRILTAG SETTINGS
    // =====================================

    static final int TAG_ID = 0; // CHANGE to your AprilTag ID

    // =====================================
    // SHOOTING POSITION
    // =====================================

    static final double TARGET_DISTANCE_IN = 0;
    static final double TARGET_SIDEWAYS_IN = 0;
    static final double TARGET_YAW_DEG = 0;

    // =====================================
    // DRIVE SETTINGS
    // =====================================

    static final double DRIVE_GAIN = 0.03;
    static final double TURN_GAIN = 0.02;

    static final double MAX_POWER = 0.4;

    static final double DONE_IN = 1.0;
    static final double DONE_DEG = 2.0;

    // =====================================
    // SHOOTER SETTINGS
    // =====================================

    static final double SHOT_SPEED_TPS = 0;
    // Replace 0 with your measured Shooter Tuning speed.

    @Override
    public void runOpMode() {

        initRobot();

        telemetry.addLine("Tag Shot Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Find the AprilTag
            Pose3D robot = robotFromTag();

            // Stop if A is released or tag is lost
            if (robot == null || !gamepad1.a) {

                drive(0, 0, 0);

                flywheel.stop();
                intake.setPower(0);
                windmill.setPower(0);

                telemetry.addData("Tag",
                        robot == null ? "Not Seen" : "Seen - Hold A");

                telemetry.update();
                continue;
            }

            // =====================================
            // READ LIMELIGHT POSITION
            // =====================================

            Position p = robot.getPosition()
                    .toUnit(DistanceUnit.INCH);

            double distance = p.z;
            double sideways = p.x;

            double yaw = robot.getOrientation()
                    .getYaw(AngleUnit.DEGREES);

            // =====================================
            // CALCULATE POSITION ERROR
            // =====================================

            double distErr =
                    distance - TARGET_DISTANCE_IN;

            double sideErr =
                    sideways - TARGET_SIDEWAYS_IN;

            double yawErr =
                    yaw - TARGET_YAW_DEG;

            // Keep turning error between -180 and 180
            while (yawErr > 180) yawErr -= 360;
            while (yawErr < -180) yawErr += 360;

            // =====================================
            // CHECK IF AT SHOOTING POSITION
            // =====================================

            boolean atSpot =
                    Math.abs(distErr) < DONE_IN
                            && Math.abs(sideErr) < DONE_IN
                            && Math.abs(yawErr) < DONE_DEG;

            // =====================================
            // DRIVE TO SHOOTING POSITION
            // =====================================

            if (atSpot) {

                drive(0, 0, 0);

            } else {

                double forward =
                        Range.clip(
                                distErr * DRIVE_GAIN,
                                -MAX_POWER,
                                MAX_POWER);

                double strafe =
                        Range.clip(
                                sideErr * DRIVE_GAIN,
                                -MAX_POWER,
                                MAX_POWER);

                double turn =
                        Range.clip(
                                yawErr * TURN_GAIN,
                                -MAX_POWER,
                                MAX_POWER);

                drive(forward, strafe, turn);
            }

            // =====================================
            // SPIN UP FLYWHEEL
            // =====================================

            flywheel.spinTo(SHOT_SPEED_TPS);

            // Only shoot when positioned and ready
            boolean shoot = atSpot && flywheel.isReady();

            // =====================================
            // FEED BALLS
            // =====================================

            intake.setPower(
                    shoot ? TuningConfig.intakePower : 0);

            windmill.setPower(
                    shoot ? TuningConfig.windmillPower : 0);

            // =====================================
            // TELEMETRY
            // =====================================

            telemetry.addData("Tag", "Detected");

            telemetry.addData("Distance", "%.1f", distance);
            telemetry.addData("Sideways", "%.1f", sideways);
            telemetry.addData("Yaw", "%.1f", yaw);

            telemetry.addData("Distance Error", "%.1f", distErr);
            telemetry.addData("Side Error", "%.1f", sideErr);
            telemetry.addData("Yaw Error", "%.1f", yawErr);

            telemetry.addData("At Shooting Spot", atSpot);

            telemetry.addData("Flywheel Ready", flywheel.isReady());

            telemetry.addData("Shooting", shoot);

            telemetry.addData(
                    "Flywheel Speed",
                    "%.0f / %.0f tps",
                    flywheel.getVelocity1(),
                    flywheel.getTarget());

            telemetry.update();
        }

        // Stop everything when OpMode ends
        drive(0, 0, 0);
        flywheel.stop();
        intake.setPower(0);
        windmill.setPower(0);

        endAuto();
    }

    // =====================================
    // LIMELIGHT APRILTAG DETECTION
    // =====================================

    private Pose3D robotFromTag() {

        LLResult result = limelight.getLatestResult();

        if (result == null
                || !result.isValid()
                || result.getStaleness() > 100) {

            return null;
        }

        for (LLResultTypes.FiducialResult tag :
                result.getFiducialResults()) {

            if (tag.getFiducialId() == TAG_ID) {

                return tag.getRobotPoseTargetSpace();
            }
        }

        return null;
    }

    // =====================================
    // MECANUM DRIVE
    // =====================================

    private void drive(
            double forward,
            double strafe,
            double turn) {

        frontLeftDrive.setPower(
                forward + strafe - turn);

        backLeftDrive.setPower(
                forward - strafe - turn);

        frontRightDrive.setPower(
                forward - strafe + turn);

        backRightDrive.setPower(
                forward + strafe + turn);
    }
}
