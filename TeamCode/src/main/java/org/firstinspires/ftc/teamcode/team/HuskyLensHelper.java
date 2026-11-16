package org.firstinspires.ftc.teamcode.team;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Wraps the HuskyLens camera (goBILDA / DFRobot) so the rest of the code never talks to it directly.
 *
 * The HuskyLens plugs into an I2C port on the Control Hub. In the Robot Configuration, add it as a
 * "HuskyLens" device named "huskylens". On the camera itself, set General Settings > Protocol Type to I2C.
 *
 * What it gives you: a list of "blocks". Each block is a box around something it recognized, with
 *   id (which tag / color / object), x and y (center, pixels), width, height, left, top.
 * The screen is 320 x 240 pixels, so x = 160 means "dead center".
 *
 * If no "huskylens" device is in the Robot Configuration, this class does nothing:
 * blocks() is empty, seesTarget() is false, and the OpMode keeps running.
 */
public class HuskyLensHelper {
    public static final String DEVICE_NAME = "huskylens";
    /** Center of the HuskyLens image, in pixels. */
    public static final int CENTER_X = 160; //*
    public static final int CENTER_Y = 120; //*

    private static final HuskyLens.Block[] NONE = new HuskyLens.Block[0];

    /** Null when no "huskylens" device is configured. Every method checks this. */
    private final HuskyLens huskyLens;
    private HuskyLens.Algorithm algorithmInUse = null;

    // Reading the camera over I2C takes a few milliseconds, so we only re-read every
    // TuningConfig.huskyLensReadMs and reuse the last answer in between.
    private HuskyLens.Block[] lastBlocks = NONE;
    private final ElapsedTime sinceRead = new ElapsedTime();
    private boolean everRead = false;

    public HuskyLensHelper(HardwareMap hardwareMap) {
        huskyLens = hardwareMap.tryGet(HuskyLens.class, DEVICE_NAME);
        applyAlgorithm();
    }

    /** True if a "huskylens" device was found in the Robot Configuration. */
    public boolean isConnected() {
        return huskyLens != null;
    }

    /** True if the camera answers over I2C. Good for an init-time telemetry check. */
    public boolean isResponding() {
        return huskyLens != null && huskyLens.knock();
    }

    /** Switches algorithm if TuningConfig.huskyLensAlgorithm changed. Cheap to call every loop. */
    public void applyAlgorithm() {
        if (huskyLens != null && TuningConfig.huskyLensAlgorithm != algorithmInUse) {
            algorithmInUse = TuningConfig.huskyLensAlgorithm;
            huskyLens.selectAlgorithm(algorithmInUse);
        }
    }

    /** Everything the camera currently sees. Never null; empty when nothing is seen. */
    public HuskyLens.Block[] blocks() {
        if (huskyLens == null) {
            return NONE;
        }
        applyAlgorithm();
        if (!everRead || sinceRead.milliseconds() >= TuningConfig.huskyLensReadMs) {
            HuskyLens.Block[] fresh = huskyLens.blocks();
            lastBlocks = fresh == null ? NONE : fresh;
            sinceRead.reset();
            everRead = true;
        }
        return lastBlocks;
    }

    /** True if the camera sees anything at all. */
    public boolean seesTarget() {
        return blocks().length > 0;
    }

    /** The first block with this id (tag number, trained color/object number), or null. */
    public HuskyLens.Block blockWithId(int id) {
        for (HuskyLens.Block b : blocks()) {
            if (b.id == id) {
                return b;
            }
        }
        return null;
    }

    /** The biggest block on screen (usually the closest thing), or null. */
    public HuskyLens.Block largestBlock() {
        HuskyLens.Block best = null;
        for (HuskyLens.Block b : blocks()) {
            if (best == null || b.width * b.height > best.width * best.height) {
                best = b;
            }
        }
        return best;
    }

    /**
     * TODO (students): use the HuskyLens to correct odometry drift.
     *
     * The HuskyLens does NOT report a field position the way the Limelight did. It only tells you
     * where a tag is in the picture (x, y, width, height). To turn that into a robot position you
     * need to work out:
     *   - which tag it is (block.id) and where that tag sits on the field,
     *   - how far away it is (bigger width = closer; measure width at a few known distances),
     *   - which way it is (block.x vs CENTER_X tells you left/right).
     *
     * Until that is written, this returns false and changes nothing, so autos still run.
     *
     * @return true if a correction was applied
     */
    public boolean relocalize(Follower follower) {
        return false;
    }

    /** Nothing to shut down on the HuskyLens; kept so OpModes can call it at the end. */
    public void stop() {
    }
}


//*
// GET robot's current X and Y position
//
//CALCULATE distance from robot to shooting target
//
//IF distance is less than 20 inches:
//    SET shooter power to 0.6
//
//ELSE IF distance is less than 30 inches:
//    SET shooter power to 0.7
//
//ELSE IF distance is less than 40 inches:
//    SET shooter power to 0.8
//
//ELSE:
//    SET shooter power to 0.9
//
//SET shooter motor to calculated power
