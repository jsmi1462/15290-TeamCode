package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Closed-loop flywheel speed using the shooter encoders. The motor controller adds power as the battery
 * sags, so the speed holds instead of dropping when the voltage dips. isReady() is true only after BOTH
 * flywheels have stayed within tolerance of the target for a short time, so you don't feed a ball early.
 * Set motor directions yourself before calling init(); velocities are direction-adjusted.
 */
public class ShooterFlywheel {
    private final DcMotorEx m1;
    private final DcMotorEx m2;
    private final ElapsedTime inBand = new ElapsedTime();
    private double target = 0;
    private boolean wasInBand = false;
    private double appliedP = Double.NaN, appliedI, appliedD, appliedF;
    private boolean appliedCustom = false;

    public ShooterFlywheel(DcMotorEx m1, DcMotorEx m2) {
        this.m1 = m1;
        this.m2 = m2;
    }

    /** Call once after setting directions. Does not reset the encoders. */
    public void init() {
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /** Set the target speed in encoder ticks per second (0 stops). Call every loop; dashboard PIDF edits apply here. */
    public void spinTo(double ticksPerSec) {
        target = Math.max(0, ticksPerSec);
        applyPidf();
        m1.setVelocity(target);
        m2.setVelocity(target);
    }

    public void stop() {
        spinTo(0);
    }

    public double getTarget() {
        return target;
    }

    public double getVelocity1() {
        return m1.getVelocity();
    }

    public double getVelocity2() {
        return m2.getVelocity();
    }

    /** True once both flywheels have held within tolerance of the target for TuningConfig.readyHoldMs. */
    public boolean isReady() {
        boolean now = target > 0
                && Math.abs(m1.getVelocity() - target) <= TuningConfig.velocityToleranceTps
                && Math.abs(m2.getVelocity() - target) <= TuningConfig.velocityToleranceTps;
        if (now && !wasInBand) {
            inBand.reset();
        }
        wasInBand = now;
        return now && inBand.milliseconds() >= TuningConfig.readyHoldMs;
    }

    private void applyPidf() {
        boolean custom = TuningConfig.useCustomPidf;
        if (custom == appliedCustom && !custom) {
            return; // leave the SDK defaults alone
        }
        if (custom && appliedCustom && appliedP == TuningConfig.velocityP && appliedI == TuningConfig.velocityI
                && appliedD == TuningConfig.velocityD && appliedF == TuningConfig.velocityF) {
            return;
        }
        if (custom) {
            PIDFCoefficients c = new PIDFCoefficients(TuningConfig.velocityP, TuningConfig.velocityI,
                    TuningConfig.velocityD, TuningConfig.velocityF);
            m1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c);
            m2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c);
            appliedP = c.p;
            appliedI = c.i;
            appliedD = c.d;
            appliedF = c.f;
        }
        appliedCustom = custom;
    }
}
