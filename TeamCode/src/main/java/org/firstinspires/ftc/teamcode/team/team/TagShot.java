
package org.firstinspires.ftc.teamcode.team.team;

import static org.firstinspires.ftc.teamcode.team.TuningConfig.shooterPower;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
    // LIMELIGHT SETTINGS
    // =====================================

    private static final int LIMELIGHT_PIPELINE = 0;

    // =====================================
    // APRILTAG IDS
    // =====================================

    private static final int TAG_31 = 31;
    private static final int TAG_44 = 44;
    private static final int TAG_36 = 36;
    private static final int TAG_39 = 39;

    // =====================================
    // DRIVE SETTINGS
    // =====================================

    private static final double DRIVE_GAIN = 0.03;
    private static final double TURN_GAIN = 0.02;

    private static final double MAX_POWER = 0.30;

    private static final double DONE_IN = 1.0;
    private static final double DONE_DEG = 2.0;

    private static final double MAX_TAG_STALENESS_MS = 100.0;

    // =====================================
    // SHOOTING POSITIONS
    // Replace zeros with measured values
    // =====================================

    // AprilTag 31
    private static final TagTarget TARGET_31 =
            new TagTarget(31, 0, 0, 0, 0);

    // AprilTag 44
    private static final TagTarget TARGET_44 =
            new TagTarget(44, 0, 0, 0, 0);

    // AprilTag 36
    private static final TagTarget TARGET_36 =
            new TagTarget(36, 0, 0, 0, 0);

    // AprilTag 39
    private static final TagTarget TARGET_39 =
            new TagTarget(39, 0, 0, 0, 0);

    private Limelight3A limelight;

    private boolean yWasPressed = false;
    private boolean xWasPressed = false;

    // =====================================
    // TAG TARGET CLASS
    // =====================================

    private static class TagTarget {

        final int id;
        final double distanceIn;
        final double sidewaysIn;
        final double yawDeg;
        final double shotSpeedTps;

        TagTarget(int id,
                  double distanceIn,
                  double sidewaysIn,
                  double yawDeg,
                  double shotSpeedTps) {

            this.id = id;
            this.distanceIn = distanceIn;
            this.sidewaysIn = sidewaysIn;
            this.yawDeg = yawDeg;
            this.shotSpeedTps = shotSpeedTps;
        }
    }

    // =====================================
    // MAIN OPMODE
    // =====================================

    @Override
    public void runOpMode() {

        initRobot();

        // Initialize Limelight
        limelight = hardwareMap.get(
                Limelight3A.class, "limelight");

        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.start();

        telemetry.addLine("Tag Shot Ready");
        telemetry.addLine("Hold Driver 1 A to activate");
        telemetry.update();

        waitForStart();

        try {

            while (opModeIsActive()) {
                // ---------- SHOOTER ----------
                // One tap of Y or X (driver 2) changes the speed by 0.05.
                if (gamepad2.y && !yWasPressed) {
                    shooterPower = Math.min(1.0, Math.round((shooterPower + 0.05) * 100) / 100.0);
                }
                if (gamepad2.x && !xWasPressed) {
                    shooterPower = Math.max(0.2, Math.round((shooterPower - 0.05) * 100) / 100.0);
                }
                yWasPressed = gamepad2.y;
                xWasPressed = gamepad2.x;

                boolean shooterStop = gamepad1.b || gamepad2.b;   // emergency stop
                double shooterSetting = shooterPower;
                // The shooter holds a steady SPEED using its encoders. 1.0 means full speed (shooterMaxTps).
                flywheel.spinTo(shooterStop ? 0.0 : shooterSetting * TuningConfig.shooterMaxTps);
                // Find a recognized AprilTag
                TagDetection detection = findKnownTag();

                // Stop if A is released or tag is lost
                if (!gamepad1.a || detection == null) {

                    stopTagShot();

                    if (!gamepad1.a) {
                        telemetry.addData(
                                "Status", "Hold A to activate");
                    } else {
                        telemetry.addData(
                                "Status", "No known tag seen");
                    }

                    telemetry.addData(
                            "Accepted IDs", "31, 44, 36, 39");

                    telemetry.update();
                    continue;
                }

                // Get selected target
                TagTarget target = detection.target;
                Pose3D robotPose = detection.robotPose;

                // =====================================
                // READ LIMELIGHT POSITION
                // =====================================

                Position p = robotPose.getPosition()
                        .toUnit(DistanceUnit.INCH);

                double distance = p.z;
                double sideways = p.x;

                double yaw = robotPose.getOrientation()
                        .getYaw(AngleUnit.DEGREES);

                // =====================================
                // CALCULATE ERRORS
                // =====================================

                double distanceError =
                        distance - target.distanceIn;

                double sidewaysError =
                        sideways - target.sidewaysIn;

                double yawError =
                        yaw - target.yawDeg;

                // Correct rotation direction
                while (yawError > 180) yawError -= 360;
                while (yawError < -180) yawError += 360;

                // =====================================
                // CHECK SHOOTING POSITION
                // =====================================

                boolean atSpot =
                        Math.abs(distanceError) < DONE_IN
                                && Math.abs(sidewaysError) < DONE_IN
                                && Math.abs(yawError) < DONE_DEG;

                // =====================================
                // DRIVE TO SHOOTING POSITION
                // =====================================

                if (atSpot) {

                    driveTagShot(0, 0, 0);

                } else {

                    double forward = Range.clip(
                            distanceError * DRIVE_GAIN,
                            -MAX_POWER,
                            MAX_POWER);

                    double strafe = Range.clip(
                            sidewaysError * DRIVE_GAIN,
                            -MAX_POWER,
                            MAX_POWER);

                    double turn = Range.clip(
                            yawError * TURN_GAIN,
                            -MAX_POWER,
                            MAX_POWER);

                    driveTagShot(forward, strafe, turn);
                }

                // =====================================
                // SPIN UP FLYWHEEL
                // =====================================

                flywheel.spinTo(shooterStop ? 0.0 :
                        shooterSetting * TuningConfig.shooterMaxTps);

                // =====================================
                // CHECK IF READY TO SHOOT
                // =====================================

                boolean shoot =
                        atSpot
                                && !shooterStop
                                && flywheel.isReady();

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

                telemetry.addData(
                        "Selected AprilTag", target.id);

                telemetry.addData(
                        "Distance / Target",
                        "%.1f / %.1f in",
                        distance, target.distanceIn);

                telemetry.addData(
                        "Sideways / Target",
                        "%.1f / %.1f in",
                        sideways, target.sidewaysIn);

                telemetry.addData(
                        "Yaw / Target",
                        "%.1f / %.1f deg",
                        yaw, target.yawDeg);

                telemetry.addData(
                        "Distance Error", "%.1f", distanceError);

                telemetry.addData(
                        "Side Error", "%.1f", sidewaysError);

                telemetry.addData(
                        "Yaw Error", "%.1f", yawError);

                telemetry.addData(
                        "At Shooting Spot", atSpot);

                telemetry.addData(
                        "Flywheel Target",
                        "%.0f TPS", target.shotSpeedTps);

                telemetry.addData(
                        "Flywheel Actual",
                        "%.0f / %.0f TPS",
                        flywheel.getVelocity1(),
                        flywheel.getVelocity2());

                telemetry.addData(
                        "Flywheel Ready",
                        flywheel.isReady());

                telemetry.addData(
                        "Feeding Balls", shoot);

                telemetry.update();
            }

        } finally {

            // Stop everything when OpMode ends
            stopTagShot();

            limelight.stop();

            endAuto();
        }
    }

    // =====================================
    // FIND APRILTAG
    // =====================================

    private TagDetection findKnownTag() {

        LLResult result = limelight.getLatestResult();

        if (result == null
                || !result.isValid()
                || result.getStaleness() > MAX_TAG_STALENESS_MS) {

            return null;
        }

        for (LLResultTypes.FiducialResult fiducial :
                result.getFiducialResults()) {

            TagTarget target =
                    targetForId(fiducial.getFiducialId());

            if (target != null) {

                Pose3D pose =
                        fiducial.getRobotPoseTargetSpace();

                if (pose != null) {

                    return new TagDetection(target, pose);
                }
            }
        }

        return null;
    }

    // =====================================
    // SELECT SHOOTING POSITION
    // =====================================

    private TagTarget targetForId(int id) {

        switch (id) {

            case TAG_31:
                return TARGET_31;

            case TAG_44:
                return TARGET_44;

            case TAG_36:
                return TARGET_36;

            case TAG_39:
                return TARGET_39;

            default:
                return null;
        }
    }

    // =====================================
    // MECANUM DRIVE
    // =====================================

    private void driveTagShot(
            double forward,
            double strafe,
            double turn) {

        double frontLeft =
                forward + strafe - turn;

        double backLeft =
                forward - strafe - turn;

        double frontRight =
                forward - strafe + turn;

        double backRight =
                forward + strafe + turn;

        // Normalize wheel powers if necessary
        double largest = Math.max(
                Math.max(Math.abs(frontLeft),
                        Math.abs(backLeft)),
                Math.max(Math.abs(frontRight),
                        Math.abs(backRight)));

        if (largest > 1.0) {

            frontLeft /= largest;
            backLeft /= largest;
            frontRight /= largest;
            backRight /= largest;
        }

        frontLeftDrive.setPower(frontLeft);
        backLeftDrive.setPower(backLeft);
        frontRightDrive.setPower(frontRight);
        backRightDrive.setPower(backRight);
    }

    // =====================================
    // STOP ALL MOTORS
    // =====================================

    private void stopTagShot() {

        driveTagShot(0, 0, 0);

        flywheel.stop();

        intake.setPower(0);
        windmill.setPower(0);
    }

    // =====================================
    // TAG DETECTION DATA
    // =====================================

    private static class TagDetection {

        final TagTarget target;
        final Pose3D robotPose;

        TagDetection(TagTarget target, Pose3D robotPose) {

            this.target = target;
            this.robotPose = robotPose;
        }
    }
}
