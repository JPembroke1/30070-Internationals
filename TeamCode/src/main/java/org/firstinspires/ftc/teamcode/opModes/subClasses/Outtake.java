package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Configurable
public class Outtake {

    public DcMotorEx outtakeMotor1;
    public DcMotorEx outtakeMotor2;

    // ─────────────────────────────────────────────────────────────────────────
    // Shooter state
    // ─────────────────────────────────────────────────────────────────────────

    public static double target = 0;
    public static double currentTPS = 0;

    public static double leftVelocity = 0;
    public static double rightVelocity = 0;

    public static double lastOutput = 0;
    public static double lastError = 0;

    public static double distanceToGoal = 0;

    // ─────────────────────────────────────────────────────────────────────────
    // Tuning notes:
    // - kS helps overcome static friction.
    // - kV provides the base power needed for the target TPS.
    // - kP corrects the difference between target TPS and current TPS.
    // - If output clips at 1.0 too often, kV or kS may be too high.
    // - If the shooter reaches speed but drops too much when balls feed, increase kP slightly.
    // - If the shooter oscillates around target, reduce kP slightly.
    // ─────────────────────────────────────────────────────────────────────────

    public static double kS = 0.08;
    public static double kV = 0.00040;
    public static double kP = 0.00035;

    public static double MIN_OUTPUT = 0.0;
    public static double MAX_OUTPUT = 1.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Linear regression target model
    //
    // targetTPS = (regressionSlope * distanceCM) + regressionIntercept
    //
    // Tuning notes:
    // - If shots are consistently low/high at all distances, tune regressionIntercept.
    // - If close shots are good but far shots are wrong, tune regressionSlope.
    // - Increasing intercept raises all shots by a similar amount.
    // - Increasing slope mainly affects longer shots more than close shots.
    //
    // Current estimate:
    // - regressionIntercept = 1085.1 was calculated for the inset goal estimate.
    // - This should put the shooter close to 1500 TPS around field position (72,72)
    //   when using the blue inset goal target around (6,138).
    // ─────────────────────────────────────────────────────────────────────────

    public static double regressionSlope = 1.75;
    public static double regressionIntercept = 1085.1;

    public static double MIN_TARGET_TPS = 0;
    public static double MAX_TARGET_TPS = 1900;

    // ─────────────────────────────────────────────────────────────────────────
    // At-speed checking
    //
    // Tuning notes:
    // - TPS_READY_TOLERANCE = 50 means a 1500 TPS target is ready between
    //   roughly 1450 and 1550 TPS.
    // - Your current Auto can still use percentage readiness through isAtSpeed(...).
    // - For more consistent shooting later, Auto can be changed to use isAtSpeedTolerance().
    // ─────────────────────────────────────────────────────────────────────────

    public static double TPS_READY_TOLERANCE = 50;

    // ─────────────────────────────────────────────────────────────────────────
    // Init
    // ─────────────────────────────────────────────────────────────────────────

    public void init(HardwareMap hardwareMap) {
        outtakeMotor1 = hardwareMap.get(DcMotorEx.class, "outtakeLeft");
        outtakeMotor2 = hardwareMap.get(DcMotorEx.class, "outtakeRight");

        outtakeMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        outtakeMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        target = 0;
        currentTPS = 0;

        leftVelocity = 0;
        rightVelocity = 0;

        lastOutput = 0;
        lastError = 0;

        distanceToGoal = 0;

        outtakeMotor1.setPower(0);
        outtakeMotor2.setPower(0);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Target setting
    // ─────────────────────────────────────────────────────────────────────────

    public void setTargetTPS(double targetTPS) {
        target = clampTarget(targetTPS);
    }

    public void startOuttaking(double targetTPS) {
        setTargetTPS(targetTPS);
    }

    public void shootNear() {
        setTargetTPS(1200);
    }

    public void shootFar() {
        setTargetTPS(1550);
    }

    public void linearRegression(double distanceCM) {
        distanceToGoal = distanceCM;
        target = clampTarget((regressionSlope * distanceCM) + regressionIntercept);
    }

    public void linearRegression(double formula, double distanceCM, double yIntercept) {
        distanceToGoal = distanceCM;
        target = clampTarget((formula * distanceCM) + yIntercept);
    }

    public void forDistance(double distanceCM) {
        linearRegression(distanceCM);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Shooter update
    //
    // Method name kept as updatePIDF() so existing Auto/TeleOp still compiles.
    //
    // This is not using the old FTCLib PIDFController anymore.
    // Internally this now uses:
    //
    // output = kS + (kV * targetTPS) + (kP * error)
    //
    // Tuning notes:
    // - If target is 0 or less, motors are stopped.
    // - Motor output is clamped between MIN_OUTPUT and MAX_OUTPUT.
    // - Both shooter motors receive the same positive output.
    // ─────────────────────────────────────────────────────────────────────────

    public void updatePIDF() {
        updateVelocityReadings();

        if (target <= 0) {
            lastError = 0;
            lastOutput = 0;

            if (outtakeMotor1 != null) {
                outtakeMotor1.setPower(0);
            }

            if (outtakeMotor2 != null) {
                outtakeMotor2.setPower(0);
            }

            return;
        }

        lastError = target - currentTPS;

        double output = kS + (kV * target) + (kP * lastError);
        output = clampPower(output);

        lastOutput = output;

        outtakeMotor1.setPower(output);
        outtakeMotor2.setPower(output);
    }

    public void updateVelocityReadings() {
        if (outtakeMotor1 == null || outtakeMotor2 == null) {
            leftVelocity = 0;
            rightVelocity = 0;
            currentTPS = 0;
            return;
        }

        leftVelocity = Math.abs(outtakeMotor1.getVelocity());
        rightVelocity = Math.abs(outtakeMotor2.getVelocity());

        currentTPS = (leftVelocity + rightVelocity) / 2.0;
    }

    public void stopOuttake() {
        target = 0;
        lastError = 0;
        lastOutput = 0;

        if (outtakeMotor1 != null) {
            outtakeMotor1.setPower(0);
        }

        if (outtakeMotor2 != null) {
            outtakeMotor2.setPower(0);
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Readiness helpers
    //
    // isAtSpeed(threshold):
    // - percentage-based check.
    // - Example: threshold 0.95 means currentTPS must be at least 95% of target.
    //
    // isAtSpeedTolerance():
    // - fixed TPS error check.
    // - Usually better for consistent shooting once tuning is stable.
    // ─────────────────────────────────────────────────────────────────────────

    public boolean isAtSpeed(double threshold) {
        return target > 0 && currentTPS >= target * threshold;
    }

    public boolean isAtSpeedTolerance() {
        return target > 0 && Math.abs(target - currentTPS) <= TPS_READY_TOLERANCE;
    }

    public boolean isAtSpeedTolerance(double toleranceTPS) {
        return target > 0 && Math.abs(target - currentTPS) <= toleranceTPS;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Getter helpers
    // ─────────────────────────────────────────────────────────────────────────

    public double getTargetTPS() {
        return target;
    }

    public double getCurrentTPS() {
        return currentTPS;
    }

    public double getLeftVelocity() {
        return leftVelocity;
    }

    public double getRightVelocity() {
        return rightVelocity;
    }

    public double getLastOutput() {
        return lastOutput;
    }

    public double getLastError() {
        return lastError;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Utility helpers
    // ─────────────────────────────────────────────────────────────────────────

    private double clampTarget(double value) {
        return clamp(value, MIN_TARGET_TPS, MAX_TARGET_TPS);
    }

    private double clampPower(double value) {
        return clamp(value, MIN_OUTPUT, MAX_OUTPUT);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}

