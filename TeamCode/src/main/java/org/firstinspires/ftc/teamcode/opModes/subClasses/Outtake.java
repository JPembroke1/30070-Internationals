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
    public static double maxTargetTPS = 1900.0;

    public static double farZoneMinTargetTPS = 1000.0;
    public static double farZoneMaxTargetTPS = 3500.0;

    public static double MAX_REGRESSION_DISTANCE_CM = 200.0;

    public static double kV = 0.00039;
    public static double kS = 0.06;
    public static double kP = 0.0007;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double minPower = 0.0;
    public static double maxPower = 1.0;

    public static double ERROR_POWER_LIMIT_BASE = 0.55;
    public static double ERROR_POWER_LIMIT_GAIN = 0.00087;

    public static double ERROR_POWER_LIMIT_MIN = 0.45;
    public static double ERROR_POWER_LIMIT_MAX = 0.95;
    public static double ERROR_POWER_LIMIT_ERROR_CAP = 1000.0;

    public static double RAMP_UP_PER_SECOND = 2.5;
    public static double RAMP_DOWN_PER_SECOND = 6.0;

    public static double MIN_VALID_TPS = 50.0;
    public static double MAX_ERROR_FOR_PID = 500.0;

    public static double target = 0.0;
    public static double requestedTarget = 0.0;
    public static double currentTPS = 0.0;
    public static double leftVelocity = 0.0;
    public static double rightVelocity = 0.0;
    public static double effectiveTPS = 0.0;

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

        double rawError = target - effectiveTPS;
        double error = clamp(rawError, -MAX_ERROR_FOR_PID, MAX_ERROR_FOR_PID);

        integral += error * dt;

        double derivative = (error - previousError) / dt;

        double feedForward = (kV * target) + kS;

        double requestedOutput =
                feedForward
                        + (kP * error)
                        + (kI * integral)
                        + (kD * derivative);

        double powerLimit = calculatePowerLimitFromError(rawError);

        requestedOutput = clamp(requestedOutput, minPower, powerLimit);

        applyRamp(requestedOutput, dt, powerLimit);

        previousError = error;
        previousUpdateTime = now;
    }

    private double calculatePowerLimitFromError(double error) {
        double cappedError = clamp(Math.abs(error), 0.0, ERROR_POWER_LIMIT_ERROR_CAP);

        double limit =
                ERROR_POWER_LIMIT_BASE
                        + (cappedError * ERROR_POWER_LIMIT_GAIN);

        return clamp(limit, ERROR_POWER_LIMIT_MIN, ERROR_POWER_LIMIT_MAX);
    }

    private void applyRamp(double requested, double dt, double limit) {
        double diff = requested - appliedPower;

        double maxStep =
                (diff > 0.0)
                        ? RAMP_UP_PER_SECOND * dt
                        : RAMP_DOWN_PER_SECOND * dt;

        appliedPower += clamp(diff, -maxStep, maxStep);
        appliedPower = clamp(appliedPower, minPower, limit);

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

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}