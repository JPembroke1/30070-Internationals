package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Outtake {

    // ─────────────────────────────────────────────────────────────────────────
    // Motors
    // ─────────────────────────────────────────────────────────────────────────

    private DcMotorEx leftShooter = null;
    private DcMotorEx rightShooter = null;

    // Change these if your robot configuration names are different.
    public static String LEFT_SHOOTER_MOTOR_NAME = "leftOuttake";
    public static String RIGHT_SHOOTER_MOTOR_NAME = "rightOuttake";

    // ─────────────────────────────────────────────────────────────────────────
    // Regression target tuning
    //
    // target TPS = regressionSlope * distanceCM + regressionIntercept
    //
    // Keep these values tuneable for match/tuning work.
    // ─────────────────────────────────────────────────────────────────────────

    public static double regressionSlope = 0.0;
    public static double regressionIntercept = 1500.0;

    public static double minTargetTPS = 800.0;
    public static double maxTargetTPS = 2300.0;

    // ─────────────────────────────────────────────────────────────────────────
    // PIDF tuning
    //
    // kV:
    // - Feedforward per TPS.
    //
    // kS:
    // - Static power needed to overcome friction.
    //
    // kP:
    // - Main correction value.
    //
    // kI:
    // - Usually keep at 0 unless you really need it.
    //
    // kD:
    // - Usually keep low or 0 for shooter velocity.
    // ─────────────────────────────────────────────────────────────────────────

    public static double kV = 0.00035;
    public static double kS = 0.04;
    public static double kP = 0.00025;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double minPower = 0.0;
    public static double maxPower = 1.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Sensor validation tuning
    //
    // MIN_VALID_TPS:
    // - Any velocity below this is treated as invalid while the shooter target
    //   is active.
    //
    // MAX_TPS_MISMATCH:
    // - If left/right differ by more than this, the side further from last known
    //   reasonable speed can be ignored.
    //
    // MAX_ERROR_FOR_PID:
    // - Limits correction error so one bad reading cannot cause a huge power spike.
    //
    // HOLD_OUTPUT_WHEN_BOTH_INVALID:
    // - If both velocity sensors look bad, hold last output instead of spiking.
    // ─────────────────────────────────────────────────────────────────────────

    public static double MIN_VALID_TPS = 50.0;
    public static double MAX_TPS_MISMATCH = 900.0;
    public static double MAX_ERROR_FOR_PID = 500.0;

    public static boolean HOLD_OUTPUT_WHEN_BOTH_INVALID = true;
    public static boolean USE_SENSOR_VALIDATION = true;

    // ─────────────────────────────────────────────────────────────────────────
    // Readiness tuning
    // ─────────────────────────────────────────────────────────────────────────

    public static double atSpeedToleranceTPS = 80.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Public telemetry values used by TeleOp / Auto
    // ─────────────────────────────────────────────────────────────────────────

    public static double target = 0.0;

    public static double currentTPS = 0.0;
    public static double leftVelocity = 0.0;
    public static double rightVelocity = 0.0;

    public static double effectiveTPS = 0.0;

    public static double lastError = 0.0;
    public static double lastOutput = 0.0;

    public static double lastFeedForward = 0.0;
    public static double lastP = 0.0;
    public static double lastI = 0.0;
    public static double lastD = 0.0;

    public static boolean leftSensorValid = false;
    public static boolean rightSensorValid = false;
    public static boolean bothSensorsValid = false;
    public static boolean usingLeftOnly = false;
    public static boolean usingRightOnly = false;
    public static boolean bothSensorsInvalid = false;

    public static double leftLastValidTPS = 0.0;
    public static double rightLastValidTPS = 0.0;

    public static double sensorMismatch = 0.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Internal PID state
    // ─────────────────────────────────────────────────────────────────────────

    private double integral = 0.0;
    private double previousError = 0.0;
    private double previousUpdateTime = 0.0;
    private boolean pidInitialised = false;

    // ─────────────────────────────────────────────────────────────────────────
    // Init
    // ─────────────────────────────────────────────────────────────────────────

    public void init(HardwareMap hardwareMap) {
        leftShooter = hardwareMap.get(DcMotorEx.class, LEFT_SHOOTER_MOTOR_NAME);
        rightShooter = hardwareMap.get(DcMotorEx.class, RIGHT_SHOOTER_MOTOR_NAME);

        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        resetController();

        stopOuttake();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Target controls
    // ─────────────────────────────────────────────────────────────────────────

    public void setTargetTPS(double targetTPS) {
        target = clamp(targetTPS, minTargetTPS, maxTargetTPS);
    }

    public void linearRegression(double distanceCM) {
        double calculatedTarget = (regressionSlope * distanceCM) + regressionIntercept;
        setTargetTPS(calculatedTarget);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Main update
    //
    // This is where sensor validation prevents the runaway spike.
    //
    // Core logic:
    // - If both sensors are valid, use average TPS.
    // - If left is invalid but right is valid, use right TPS only.
    // - If right is invalid but left is valid, use left TPS only.
    // - If both are invalid, hold last output or stop safely.
    // ─────────────────────────────────────────────────────────────────────────

    public void updatePIDF() {
        if (leftShooter == null || rightShooter == null) {
            return;
        }

        readVelocities();
        validateSensors();
        chooseEffectiveTPS();

        if (target <= 0.0) {
            stopOuttake();
            return;
        }

        if (bothSensorsInvalid && HOLD_OUTPUT_WHEN_BOTH_INVALID) {
            setShooterPower(lastOutput);
            return;
        }

        double now = getTimeSeconds();
        double dt;

        if (!pidInitialised) {
            previousUpdateTime = now;
            previousError = 0.0;
            integral = 0.0;
            pidInitialised = true;
            dt = 0.02;
        } else {
            dt = now - previousUpdateTime;
            if (dt <= 0.001) {
                dt = 0.02;
            }
        }

        double rawError = target - effectiveTPS;
        double error = clamp(rawError, -MAX_ERROR_FOR_PID, MAX_ERROR_FOR_PID);

        integral += error * dt;

        // Basic anti-windup.
        integral = clamp(integral, -1000.0, 1000.0);

        double derivative = (error - previousError) / dt;

        double feedForward = (kV * target) + kS;
        double pTerm = kP * error;
        double iTerm = kI * integral;
        double dTerm = kD * derivative;

        double output = feedForward + pTerm + iTerm + dTerm;
        output = clamp(output, minPower, maxPower);

        setShooterPower(output);

        currentTPS = effectiveTPS;

        lastError = rawError;
        lastOutput = output;

        lastFeedForward = feedForward;
        lastP = pTerm;
        lastI = iTerm;
        lastD = dTerm;

        previousError = error;
        previousUpdateTime = now;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Velocity reading
    // ─────────────────────────────────────────────────────────────────────────

    private void readVelocities() {
        leftVelocity = safeVelocity(leftShooter);
        rightVelocity = safeVelocity(rightShooter);

        sensorMismatch = Math.abs(leftVelocity - rightVelocity);
    }

    private double safeVelocity(DcMotorEx motor) {
        if (motor == null) {
            return 0.0;
        }

        double velocity = motor.getVelocity();

        if (Double.isNaN(velocity) || Double.isInfinite(velocity)) {
            return 0.0;
        }

        return Math.abs(velocity);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Sensor validation
    // ─────────────────────────────────────────────────────────────────────────

    private void validateSensors() {
        if (!USE_SENSOR_VALIDATION) {
            leftSensorValid = true;
            rightSensorValid = true;
            bothSensorsValid = true;
            bothSensorsInvalid = false;
            return;
        }

        leftSensorValid = leftVelocity >= MIN_VALID_TPS;
        rightSensorValid = rightVelocity >= MIN_VALID_TPS;

        if (leftSensorValid) {
            leftLastValidTPS = leftVelocity;
        }

        if (rightSensorValid) {
            rightLastValidTPS = rightVelocity;
        }

        // If both are technically valid but wildly different, reject the one
        // that is further from the current target.
        if (leftSensorValid && rightSensorValid && sensorMismatch > MAX_TPS_MISMATCH) {
            double leftErrorToTarget = Math.abs(target - leftVelocity);
            double rightErrorToTarget = Math.abs(target - rightVelocity);

            if (leftErrorToTarget > rightErrorToTarget) {
                leftSensorValid = false;
            } else {
                rightSensorValid = false;
            }
        }

        bothSensorsValid = leftSensorValid && rightSensorValid;
        bothSensorsInvalid = !leftSensorValid && !rightSensorValid;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Effective TPS selection
    // ─────────────────────────────────────────────────────────────────────────

    private void chooseEffectiveTPS() {
        usingLeftOnly = false;
        usingRightOnly = false;

        if (bothSensorsValid) {
            effectiveTPS = (leftVelocity + rightVelocity) / 2.0;
            currentTPS = effectiveTPS;
            return;
        }

        if (leftSensorValid && !rightSensorValid) {
            effectiveTPS = leftVelocity;
            currentTPS = effectiveTPS;
            usingLeftOnly = true;
            return;
        }

        if (rightSensorValid && !leftSensorValid) {
            effectiveTPS = rightVelocity;
            currentTPS = effectiveTPS;
            usingRightOnly = true;
            return;
        }

        // Both invalid.
        // This prevents the controller from thinking TPS is 0 and spiking power.
        bothSensorsInvalid = true;

        if (HOLD_OUTPUT_WHEN_BOTH_INVALID) {
            effectiveTPS = target;
        } else {
            effectiveTPS = 0.0;
        }

        currentTPS = effectiveTPS;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Stop / reset
    // ─────────────────────────────────────────────────────────────────────────

    public void stopOuttake() {
        target = 0.0;

        setShooterPower(0.0);

        currentTPS = 0.0;
        effectiveTPS = 0.0;

        lastError = 0.0;
        lastOutput = 0.0;

        lastFeedForward = 0.0;
        lastP = 0.0;
        lastI = 0.0;
        lastD = 0.0;

        resetController();
    }

    private void resetController() {
        integral = 0.0;
        previousError = 0.0;
        previousUpdateTime = getTimeSeconds();
        pidInitialised = false;
    }

    private void setShooterPower(double power) {
        double clipped = clamp(power, minPower, maxPower);

        if (leftShooter != null) {
            leftShooter.setPower(clipped);
        }

        if (rightShooter != null) {
            rightShooter.setPower(clipped);
        }

        lastOutput = clipped;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Speed readiness helpers
    // ─────────────────────────────────────────────────────────────────────────

    public boolean isAtSpeed(double percentOfTarget) {
        if (target <= 0.0) {
            return false;
        }

        if (bothSensorsInvalid) {
            return false;
        }

        return effectiveTPS >= target * percentOfTarget;
    }

    public boolean isAtSpeedTolerance() {
        if (target <= 0.0) {
            return false;
        }

        if (bothSensorsInvalid) {
            return false;
        }

        return Math.abs(target - effectiveTPS) <= atSpeedToleranceTPS;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Utility
    // ─────────────────────────────────────────────────────────────────────────

    private double getTimeSeconds() {
        return System.nanoTime() / 1_000_000_000.0;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}