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
    /** false = plain power control (Y/X taps scale motor power directly). true = encoder velocity control. Leave false until ShooterTuning shows both velocities positive and tracking. */
    public static boolean useVelocityControl = false;
    /** Full-power speed in ticks/sec. Power 1.0 = this speed. Measure it: run ShooterTuning at shooterPower 1.0 and read the velocity. */
    public static double shooterMaxTps = 2000;
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

    // ---- Shooting tuning (ShooterTuning OpMode only) ----
    /** Tolerance, in ticks per second, for "at speed" on the tuning screen. */
    public static double velocityToleranceTps = 50;
}
