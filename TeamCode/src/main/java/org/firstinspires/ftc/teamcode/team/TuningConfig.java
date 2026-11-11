package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.config.Config;

/**
 * Values you can change live from FTC Dashboard (http://192.168.43.1:8080/dash while on the robot's Wi-Fi).
 * Edit a value in the Configuration tab and press Enter; it takes effect on the next loop.
 * Dashboard edits are NOT saved: when the robot app restarts these defaults come back, so copy good numbers here.
 */
@Config
public class TuningConfig {
    // ---- Shooter speed ----
    /** When true, DriveMode ignores the Y/X taps and uses shooterPower below. */
    public static boolean overrideShooterPower = false;
    public static double shooterPower = 0.7;
    /** AutoUp flywheel power and how long it runs before driving. */
    public static double autoShooterPower = 0.7;
    public static int autoShooterMs = 15000;

    // ---- Flywheel speed control (uses the shooter encoders) ----
    /** true = the shooter uses its encoders to hold a steady speed (normal). false = plain motor power, only if an encoder cable is broken. */
    // If this ever runs away to full power again, check that both shooter velocities read POSITIVE in Shooter Tuning.
    // (2026-11-11: they read negative because the two shooter encoder cables were swapped at the hub.)
    public static boolean useVelocityControl = true;
    /** With speed control off there is no speed reading, so "ready" just waits this long after the flywheel starts. */
    public static double openLoopSpinUpMs = 1500;
    /** The speed (ticks/sec) that a shooter setting of 1.0 asks for. The Y/X taps pick a setting from 0.2 to 1.0, and speed = setting x this number.
     *  The goBILDA 5203 (6000 RPM, 28 ticks per rev) spins freely at 2800, but it cannot hold that under load,
     *  so this is set lower (about 85%) so every setting is reachable. Check it: run ShooterTuning at shooterPower 1.0
     *  and make sure READY shows. If READY never shows, lower this number. If there is room to spare, raise it (max 2800). */
    public static double shooterMaxTps = 2400;
    /** Both flywheels must be within velocityToleranceTps of target for this long to count as ready. */
    public static double readyHoldMs = 150;
    /** Max seconds AutoUp waits for the flywheels to spool up. */
    public static double autoSpoolTimeoutS = 5.0;
    /** Off = SDK default velocity PIDF. On = use the numbers below (turning it back off needs an OpMode restart). */
    public static boolean useCustomPidf = false;
    public static double velocityP = 10;
    public static double velocityI = 3;
    public static double velocityD = 0;
    public static double velocityF = 12;

    // ---- Other motors ----
    public static double intakePower = 1.0;
    public static double windmillPower = 1.0;

    // ---- Limelight ----
    public static int limelightPipeline = 0;
    /** How many results per second the Limelight sends back (1-250). */
    public static int limelightPollHz = 100;

    // ---- Corner reset (two distance sensors on the back-right corner: one faces back, one faces right) ----
    public static boolean cornerResetEnabled = true;
    /** Both sensors must read this (inches) to count as "in the corner". */
    public static double cornerN = 2.0;
    /** How close to cornerN each reading must be (inches). */
    public static double cornerTolIn = 0.75;
    /** Readings must stay in range this long before the reset fires. */
    public static double cornerHoldMs = 200;
    /** Pinpoint pose to load when the reset fires (raw Pinpoint inches / degrees). */
    public static double cornerX = 0;
    public static double cornerY = 0;
    public static double cornerHeadingDeg = 0;

    // ---- Snap-to heading (gamepad1 triggers) ----
    /** Left trigger snaps to this heading, right trigger to the other. Degrees, Pinpoint heading. */
    public static double snapLeftDeg = 45;
    public static double snapRightDeg = -45;
    public static double snapKp = 0.03;      // yaw power per degree of error
    public static double snapMaxPower = 0.6;
    public static double snapDoneDeg = 1.5;
    /** Flip to -1 if the robot turns away from the target instead of toward it. */
    public static double snapYawSign = 1;

    // ---- Shooting tuning (ShooterTuning OpMode only) ----
    /** Tolerance, in ticks per second, for "at speed" on the tuning screen. */
    public static double velocityToleranceTps = 50;
}
