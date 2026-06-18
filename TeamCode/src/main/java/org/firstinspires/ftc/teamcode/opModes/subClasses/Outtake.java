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

    // TUNE TIP:
    // These names must exactly match the Robot Controller configuration.
    public static String LEFT_SHOOTER_MOTOR_NAME = "leftOuttake";
    public static String RIGHT_SHOOTER_MOTOR_NAME = "rightOuttake";

    // ─────────────────────────────────────────────────────────────────────────
    // Regression target tuning
    //
    // target TPS = regressionSlope * distanceCM + regressionIntercept
    // ─────────────────────────────────────────────────────────────────────────

    public static double regressionSlope = 1.5;
    public static double regressionIntercept = 1560.0;

    public static double minTargetTPS = 800.0;
    public static double maxTargetTPS = 1900.0;

    // ─────────────────────────────────────────────────────────────────────────
    // PIDF tuning
    //
    // TUNE ORDER:
    // 1. kV first
    // 2. kS second
    // 3. kP third
    // 4. Leave kI and kD at 0 unless needed
    // ─────────────────────────────────────────────────────────────────────────

    public static double kV = 0.00038;
    public static double kS = 0.06;
    public static double kP = 0.0004;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double minPower = 0.0;
    public static double maxPower = 1.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Startup tuning
    //
    // This fixes the deadlock where both encoders read 0 TPS before the motors
    // have started spinning.
    //
    // If both sensors are invalid and the shooter has a target, this power is
    // used to get the flywheels moving.
    // ─────────────────────────────────────────────────────────────────────────

    public static double STARTUP_MIN_POWER = 0.45;
    public static double BOTH_INVALID_POWER_LIMIT = 0.75;

    // ─────────────────────────────────────────────────────────────────────────
    // Sensor validation tuning
    //
    // MIN_VALID_TPS:
    // - Any velocity below this is considered not useful for closed-loop PID.
    //
    // MAX_TPS_MISMATCH:
    // - If left and right differ too much, the worse side can be ignored.
    //
    // MAX_ERROR_FOR_PID:
    // - Limits correction so bad readings do not cause huge spikes.
    // ─────────────────────────────────────────────────────────────────────────

    public static double MIN_VALID_TPS = 50.0;
    public static double MAX_TPS_MISMATCH = 900.0;
    public static double MAX_ERROR_FOR_PID = 500.0;

    public static boolean USE_SENSOR_VALIDATION = true;

    // If true, the code uses a safe open-loop output when both sensors are invalid.
    // This is safer than pretending the shooter is already at target.
    public static boolean USE_OPEN_LOOP_WHEN_BOTH_INVALID = true;

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

    // Extra diagnostics
    public static boolean usingOpenLoopStartup = false;
    public static boolean hasEverSeenValidSensor = false;

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

        leftShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        double previousTarget = target;

        target = clamp(targetTPS, minTargetTPS, maxTargetTPS);

        if (previousTarget <= 0.0 && target > 0.0) {
            resetController();
        }
    }

    public void linearRegression(double distanceCM) {
        double calculatedTarget = (regressionSlope * distanceCM) + regressionIntercept;
        setTargetTPS(calculatedTarget);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Main update
    //
    // Important fix:
    // - Do not hold lastOutput when both sensors are invalid at startup.
    // - At startup, lastOutput is 0, so holding it prevents the shooter from
    //   ever spinning.
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

        if (bothSensorsInvalid && USE_OPEN_LOOP_WHEN_BOTH_INVALID) {
            runOpenLoopStartup();
            return;
        }

        usingOpenLoopStartup = false;

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
    // Open-loop startup
    //
    // Used when both velocity readings are invalid.
    //
    // This should make the shooter physically spin even before encoder velocity
    // becomes valid.
    // ─────────────────────────────────────────────────────────────────────────

    private void runOpenLoopStartup() {
        usingOpenLoopStartup = true;

        double feedForward = (kV * target) + kS;
        double startupPower = Math.max(feedForward, STARTUP_MIN_POWER);

        startupPower = clamp(startupPower, minPower, BOTH_INVALID_POWER_LIMIT);

        setShooterPower(startupPower);

        currentTPS = 0.0;
        effectiveTPS = 0.0;

        lastError = target;
        lastOutput = startupPower;

        lastFeedForward = feedForward;
        lastP = 0.0;
        lastI = 0.0;
        lastD = 0.0;

        resetController();
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
            hasEverSeenValidSensor = true;
            return;
        }

        leftSensorValid = leftVelocity >= MIN_VALID_TPS;
        rightSensorValid = rightVelocity >= MIN_VALID_TPS;

        if (leftSensorValid) {
            leftLastValidTPS = leftVelocity;
            hasEverSeenValidSensor = true;
        }

        if (rightSensorValid) {
            rightLastValidTPS = rightVelocity;
            hasEverSeenValidSensor = true;
        }

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

        bothSensorsInvalid = true;
        effectiveTPS = 0.0;
        currentTPS = 0.0;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Stop / reset
    // ─────────────────────────────────────────────────────────────────────────

    public void stopOuttake() {
        target = 0.0;

        setShooterPower(0.0);

        currentTPS = 0.0;
        effectiveTPS = 0.0;

        leftVelocity = 0.0;
        rightVelocity = 0.0;

        lastError = 0.0;
        lastOutput = 0.0;

        lastFeedForward = 0.0;
        lastP = 0.0;
        lastI = 0.0;
        lastD = 0.0;

        usingOpenLoopStartup = false;

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