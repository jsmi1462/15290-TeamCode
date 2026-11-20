
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
    private static final double MAX_TAG_STALENESS_MS = 100.0;

    // =====================================
    // PREFERRED APRILTAG IDS
    // =====================================

    // RED SCORING: 30
    // RED AUDIENCE: 37
    // BLUE AUDIENCE: 38
    // BLUE SCORING: 45

    private static final int TAG_RED_SCORING = 30;
    private static final int TAG_RED_AUDIENCE = 37;
    private static final int TAG_BLUE_AUDIENCE = 38;
    private static final int TAG_BLUE_SCORING = 45;

    // =====================================
    // DRIVE SETTINGS
    // =====================================

    private static final double DRIVE_GAIN = 0.03;
    private static final double TURN_GAIN = 0.02;

    private static final double MAX_POWER = 0.30;

    private static final double DONE_IN = 1.0;
    private static final double DONE_DEG = 2.0;

    // =====================================
    // TARGET CALIBRATION
    // =====================================

    // Keep false until all four targets have
    // been measured at the desired shooting position.

    private static final boolean TARGETS_CALIBRATED = false;

    private static final TagTarget TARGET_30 =
            new TagTarget(30, 0, 0, 0, 0);

    private static final TagTarget TARGET_37 =
            new TagTarget(37, 0, 0, 0, 0);

    private static final TagTarget TARGET_38 =
            new TagTarget(38, 0, 0, 0, 0);

    private static final TagTarget TARGET_45 =
            new TagTarget(45, 0, 0, 0, 0);

    // =====================================
    // ROBOT VARIABLES
    // =====================================

    private Limelight3A limelight;

    private boolean yWasPressed = false;
    private boolean xWasPressed = false;

    private String visibleTagIds = "None";

    // =====================================
    // TAG TARGET CLASS
    // =====================================

    private static class TagTarget {

        final int id;
        final double distanceIn;
        final double sidewaysIn;
        final double yawDeg;
        final double shotSpeedTps;

        TagTarget(
                int id,
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
    // TAG DETECTION CLASS
    // =====================================

    private static class TagDetection {

        final TagTarget target;
        final Pose3D robotPose;

        TagDetection(TagTarget target, Pose3D robotPose) {

            this.target = target;
            this.robotPose = robotPose;
        }
    }

    // =====================================
    // MAIN OPMODE
    // =====================================

    @Override
    public void runOpMode() {

        initRobot();

        limelight = hardwareMap.get(
                Limelight3A.class, "limelight");

        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.start();

        telemetry.addLine("BIOBUZZ TAG SHOT READY");
        telemetry.addLine("Hold Driver 1 A to activate");
        telemetry.addLine("Preferred tags: 30, 37, 38, 45");
        telemetry.update();

        waitForStart();

        try {

            while (opModeIsActive()) {

                // =====================================
                // MANUAL SHOOTER SPEED ADJUSTMENT
                // =====================================

                if (gamepad2.y && !yWasPressed) {

                    shooterPower = Math.min(
                            1.0,
                            Math.round(
                                    (shooterPower + 0.05) * 100) / 100.0);
                }

                if (gamepad2.x && !xWasPressed) {

                    shooterPower = Math.max(
                            0.2,
                            Math.round(
                                    (shooterPower - 0.05) * 100) / 100.0);
                }

                yWasPressed = gamepad2.y;
                xWasPressed = gamepad2.x;

                boolean shooterStop =
                        gamepad1.b || gamepad2.b;

                double shooterSetting = shooterPower;

                // Encoder-based flywheel control
                flywheel.spinTo(
                        shooterStop ? 0.0 :
                                shooterSetting * TuningConfig.shooterMaxTps);

                // =====================================
                // FIND PREFERRED APRILTAG
                // =====================================

                TagDetection detection = findPreferredTag();

                // =====================================
                // STOP IF A IS RELEASED OR TAG IS LOST
                // =====================================

                if (!gamepad1.a || detection == null) {

                    stopTagShot();

                    if (!gamepad1.a) {

                        telemetry.addData(
                                "Status", "Hold A to activate");

                    } else {

                        telemetry.addData(
                                "Status", "No preferred corner tag seen");
                    }

                    telemetry.addData(
                            "Visible AprilTags", visibleTagIds);

                    telemetry.addData(
                            "Preferred IDs", "30, 37, 38, 45");

                    telemetry.addData(
                            "Calibration Complete", TARGETS_CALIBRATED);

                    telemetry.update();

                    continue;
                }

                // =====================================
                // SELECTED TARGET
                // =====================================

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

                while (yawError > 180) {
                    yawError -= 360;
                }

                while (yawError < -180) {
                    yawError += 360;
                }

                // =====================================
                // CHECK SHOOTING POSITION
                // =====================================

                boolean atSpot =
                        TARGETS_CALIBRATED
                                && Math.abs(distanceError) < DONE_IN
                                && Math.abs(sidewaysError) < DONE_IN
                                && Math.abs(yawError) < DONE_DEG;

                // =====================================
                // DRIVE TO SHOOTING POSITION
                // =====================================

                if (!TARGETS_CALIBRATED) {

                    driveTagShot(0, 0, 0);

                } else if (atSpot) {

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
                // CHECK IF READY TO SHOOT
                // =====================================

                boolean shoot =
                        TARGETS_CALIBRATED
                                && atSpot
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

                telemetry.addLine("===== BIOBUZZ TAG SHOT =====");

                telemetry.addData(
                        "Visible AprilTags", visibleTagIds);

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
                        "Distance Error",
                        "%.1f", distanceError);

                telemetry.addData(
                        "Side Error",
                        "%.1f", sidewaysError);

                telemetry.addData(
                        "Yaw Error",
                        "%.1f", yawError);

                telemetry.addData(
                        "At Shooting Spot", atSpot);

                telemetry.addData(
                        "Calibration Complete",
                        TARGETS_CALIBRATED);

                telemetry.addData(
                        "Flywheel Setting",
                        "%.2f", shooterSetting);

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

                // =====================================
                // CALIBRATION DATA
                // =====================================

                telemetry.addLine("===== COPY TARGET DATA =====");

                telemetry.addData(
                        "AprilTag ID", target.id);

                telemetry.addData(
                        "COPY DISTANCE (p.z)",
                        "%.2f", distance);

                telemetry.addData(
                        "COPY SIDEWAYS (p.x)",
                        "%.2f", sideways);

                telemetry.addData(
                        "COPY HEADING",
                        "%.2f", yaw);

                telemetry.addLine("============================");

                if (!TARGETS_CALIBRATED) {

                    telemetry.addLine(
                            "CALIBRATION REQUIRED - DRIVE DISABLED");
                }

                telemetry.update();
            }

        } finally {

            stopTagShot();

            limelight.stop();

            endAuto();
        }
    }

    // =====================================
    // FIND PREFERRED APRILTAG
    // =====================================

    private TagDetection findPreferredTag() {

        LLResult result = limelight.getLatestResult();

        visibleTagIds = "None";

        if (result == null
                || !result.isValid()
                || result.getStaleness() > MAX_TAG_STALENESS_MS) {

            visibleTagIds = "No fresh result";
            return null;
        }

        StringBuilder ids = new StringBuilder();

        TagDetection selectedDetection = null;

        for (LLResultTypes.FiducialResult fiducial :
                result.getFiducialResults()) {

            int id = fiducial.getFiducialId();

            // Record every detected BIOBUZZ AprilTag.
            if (id >= 30 && id <= 45) {

                if (ids.length() > 0) {
                    ids.append(", ");
                }

                ids.append(id);
            }

            // Only use the four preferred corner tags.
            TagTarget target = targetForId(id);

            if (target != null && selectedDetection == null) {

                Pose3D pose =
                        fiducial.getRobotPoseTargetSpace();

                if (pose != null) {

                    selectedDetection =
                            new TagDetection(target, pose);
                }
            }
        }

        visibleTagIds =
                ids.length() > 0 ? ids.toString() : "None";

        return selectedDetection;
    }

    // =====================================
    // SELECT PREFERRED CORNER TAG
    // =====================================

    private TagTarget targetForId(int id) {

        switch (id) {

            case TAG_RED_SCORING:
                return TARGET_30;

            case TAG_RED_AUDIENCE:
                return TARGET_37;

            case TAG_BLUE_AUDIENCE:
                return TARGET_38;

            case TAG_BLUE_SCORING:
                return TARGET_45;

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

        double largest = Math.max(
                Math.max(
                        Math.abs(frontLeft),
                        Math.abs(backLeft)),
                Math.max(
                        Math.abs(frontRight),
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
}
