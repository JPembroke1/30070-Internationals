package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Outtake {

    private DcMotorEx leftShooter = null;
    private DcMotorEx rightShooter = null;

    public static String LEFT_SHOOTER_MOTOR_NAME = "leftOuttake";
    public static String RIGHT_SHOOTER_MOTOR_NAME = "rightOuttake";

    public static double regressionSlope = 1.5;
    public static double regressionIntercept = 1700.0;

    public static double minTargetTPS = 1000.0;
    public static double maxTargetTPS = 1790.0;

    public static double farZoneTargetTPS = 2750.0;
    public static double farZoneMinTargetTPS = 1000.0;
    public static double farZoneMaxTargetTPS = 3500.0;

    public static double MAX_REGRESSION_DISTANCE_CM = 200.0;

    /*
     * Normal / close-zone feedforward.
     */
    public static double kV = 0.00039;

    /*
     * Far-zone feedforward.
     */
    public static double farZoneKV = 0.0005;

    public static double kS = 0.06;
    public static double kP = 0.0007;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double minPower = 0.0;
    public static double maxPower = 1.0;

    /*
     * Normal / close-zone power limiting.
     */
    public static double ERROR_POWER_LIMIT_BASE = 0.55;
    public static double ERROR_POWER_LIMIT_GAIN = 0.00087;
    public static double ERROR_POWER_LIMIT_MIN = 0.45;
    public static double ERROR_POWER_LIMIT_MAX = 0.95;
    public static double ERROR_POWER_LIMIT_ERROR_CAP = 1000.0;

    /*
     * Far-zone power limiting.
     *
     * These are separate so far-zone spin-up can be more aggressive
     * without affecting close-zone shooting.
     */
    public static double farZoneErrorPowerLimitBase = 0.65;
    public static double farZoneErrorPowerLimitGain = 0.00100;
    public static double farZoneErrorPowerLimitMin = 0.55;
    public static double farZoneErrorPowerLimitMax = 1.00;
    public static double farZoneErrorPowerLimitErrorCap = 1200.0;

    /*
     * Normal / close-zone ramping.
     */
    public static double RAMP_UP_PER_SECOND = 2.5;
    public static double RAMP_DOWN_PER_SECOND = 6.0;

    /*
     * Far-zone ramping.
     *
     * Higher ramp-up helps the shooter reach far-zone TPS faster.
     */
    public static double farZoneRampUpPerSecond = 8.0;
    public static double farZoneRampDownPerSecond = 8.0;

    public static double MIN_VALID_TPS = 50.0;
    public static double MAX_ERROR_FOR_PID = 500.0;

    public static double target = 0.0;
    public static double requestedTarget = 0.0;
    public static double currentTPS = 0.0;
    public static double leftVelocity = 0.0;
    public static double rightVelocity = 0.0;
    public static double effectiveTPS = 0.0;

    public static double activeKV = 0.0;
    public static double activeRampUpPerSecond = 0.0;
    public static double activeRampDownPerSecond = 0.0;
    public static double activePowerLimit = 0.0;

    public static double feedForwardOutput = 0.0;
    public static double requestedPowerOutput = 0.0;
    public static double appliedPowerOutput = 0.0;
    public static double shooterError = 0.0;

    public static boolean bothSensorsInvalid = false;
    public static boolean usingFarZoneTarget = false;

    private double appliedPower = 0.0;

    private double integral = 0.0;
    private double previousError = 0.0;
    private double previousUpdateTime = 0.0;
    private boolean pidInitialised = false;

    public void init(HardwareMap hardwareMap) {
        leftShooter = hardwareMap.get(DcMotorEx.class, LEFT_SHOOTER_MOTOR_NAME);
        rightShooter = hardwareMap.get(DcMotorEx.class, RIGHT_SHOOTER_MOTOR_NAME);

        leftShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        resetController();
        stopOuttake();
    }

    public void setTargetTPS(double targetTPS) {
        double previousTarget = target;

        requestedTarget = targetTPS;
        target = clamp(targetTPS, minTargetTPS, maxTargetTPS);
        usingFarZoneTarget = false;

        if (previousTarget <= 0.0 && target > 0.0) {
            resetController();
        }
    }

    public void setFarZoneTargetTPS(double targetTPS) {
        double previousTarget = target;

        requestedTarget = targetTPS;
        target = clamp(targetTPS, farZoneMinTargetTPS, farZoneMaxTargetTPS);
        usingFarZoneTarget = true;

        if (previousTarget <= 0.0 && target > 0.0) {
            resetController();
        }
    }

    public void linearRegression(double distanceCM) {
        double clampedDistance = clamp(distanceCM, 0.0, MAX_REGRESSION_DISTANCE_CM);

        double calculatedTarget =
                (regressionSlope * clampedDistance) + regressionIntercept;

        setTargetTPS(calculatedTarget);
    }

    public void updatePIDF() {
        if (leftShooter == null || rightShooter == null) {
            return;
        }

        readVelocities();

        if (target <= 0.0) {
            stopOuttake();
            return;
        }

        double now = getTimeSeconds();
        double dt = (!pidInitialised) ? 0.02 : Math.max(now - previousUpdateTime, 0.02);

        pidInitialised = true;

        activeKV = getActiveKV();
        activeRampUpPerSecond = getActiveRampUpPerSecond();
        activeRampDownPerSecond = getActiveRampDownPerSecond();

        double rawError = target - effectiveTPS;
        double error = clamp(rawError, -MAX_ERROR_FOR_PID, MAX_ERROR_FOR_PID);

        shooterError = rawError;

        integral += error * dt;

        double derivative = (error - previousError) / dt;

        double feedForward =
                (activeKV * target) + kS;

        feedForwardOutput = feedForward;

        double requestedOutput =
                feedForward
                        + (kP * error)
                        + (kI * integral)
                        + (kD * derivative);

        double powerLimit = calculatePowerLimitFromError(rawError);

        activePowerLimit = powerLimit;

        requestedOutput = clamp(requestedOutput, minPower, powerLimit);
        requestedPowerOutput = requestedOutput;

        applyRamp(requestedOutput, dt, powerLimit);

        previousError = error;
        previousUpdateTime = now;
    }

    private double getActiveKV() {
        if (usingFarZoneTarget) {
            return farZoneKV;
        }

        return kV;
    }

    private double getActiveRampUpPerSecond() {
        if (usingFarZoneTarget) {
            return farZoneRampUpPerSecond;
        }

        return RAMP_UP_PER_SECOND;
    }

    private double getActiveRampDownPerSecond() {
        if (usingFarZoneTarget) {
            return farZoneRampDownPerSecond;
        }

        return RAMP_DOWN_PER_SECOND;
    }

    private double calculatePowerLimitFromError(double error) {
        double cappedError;
        double limit;

        if (usingFarZoneTarget) {
            cappedError = clamp(
                    Math.abs(error),
                    0.0,
                    farZoneErrorPowerLimitErrorCap
            );

            limit =
                    farZoneErrorPowerLimitBase
                            + (cappedError * farZoneErrorPowerLimitGain);

            return clamp(
                    limit,
                    farZoneErrorPowerLimitMin,
                    farZoneErrorPowerLimitMax
            );
        }

        cappedError = clamp(
                Math.abs(error),
                0.0,
                ERROR_POWER_LIMIT_ERROR_CAP
        );

        limit =
                ERROR_POWER_LIMIT_BASE
                        + (cappedError * ERROR_POWER_LIMIT_GAIN);

        return clamp(
                limit,
                ERROR_POWER_LIMIT_MIN,
                ERROR_POWER_LIMIT_MAX
        );
    }

    private void applyRamp(double requested, double dt, double limit) {
        double diff = requested - appliedPower;

        double maxStep =
                (diff > 0.0)
                        ? activeRampUpPerSecond * dt
                        : activeRampDownPerSecond * dt;

        appliedPower += clamp(diff, -maxStep, maxStep);
        appliedPower = clamp(appliedPower, minPower, limit);

        appliedPowerOutput = appliedPower;

        setShooterPower(appliedPower);
    }

    private void readVelocities() {
        leftVelocity = Math.abs(leftShooter.getVelocity());
        rightVelocity = Math.abs(rightShooter.getVelocity());

        effectiveTPS = (leftVelocity + rightVelocity) / 2.0;
        currentTPS = effectiveTPS;

        bothSensorsInvalid = effectiveTPS < MIN_VALID_TPS;
    }

    public void stopOuttake() {
        target = 0.0;
        requestedTarget = 0.0;
        appliedPower = 0.0;

        activeKV = 0.0;
        activeRampUpPerSecond = 0.0;
        activeRampDownPerSecond = 0.0;
        activePowerLimit = 0.0;

        feedForwardOutput = 0.0;
        requestedPowerOutput = 0.0;
        appliedPowerOutput = 0.0;
        shooterError = 0.0;

        usingFarZoneTarget = false;

        setShooterPower(0.0);

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

        leftShooter.setPower(clipped);
        rightShooter.setPower(clipped);
    }

    public boolean isAtSpeed(double percentOfTarget) {
        if (target <= 0.0) {
            return false;
        }

        if (bothSensorsInvalid) {
            return false;
        }

        return effectiveTPS >= target * percentOfTarget;
    }

    private double getTimeSeconds() {
        return System.nanoTime() / 1_000_000_000.0;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}