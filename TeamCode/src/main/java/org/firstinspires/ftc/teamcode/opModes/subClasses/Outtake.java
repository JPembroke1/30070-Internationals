package org.firstinspires.ftc.teamcode.opModes.subClasses;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Outtake {

    public DcMotorEx outtakeMotor1;
    public DcMotorEx outtakeMotor2;
    public PIDFController controller;

    public static double target = 0;
    public static double currentTPS = 0;

    public static double leftVelocity = 0;
    public static double rightVelocity = 0;
    public static double lastOutput = 0;

    public static double P = 0.00035;
    public static double I = 0.0;
    public static double D = 0.00001;
    public static double F = 0.00042;

    public static double regressionSlope = 1.75;
    public static double regressionIntercept = 1047.4;

    public static double MIN_TARGET_TPS = 0;
    public static double MAX_TARGET_TPS = 1900;

    public static double distanceToGoal = 0;

    public void init(@NonNull HardwareMap hardwareMap) {
        outtakeMotor1 = hardwareMap.get(DcMotorEx.class, "outtakeLeft");
        outtakeMotor2 = hardwareMap.get(DcMotorEx.class, "outtakeRight");

        outtakeMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        outtakeMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        controller = new PIDFController(P, I, D, F);

        target = 0;
        currentTPS = 0;
        leftVelocity = 0;
        rightVelocity = 0;
        lastOutput = 0;

        outtakeMotor1.setPower(0);
        outtakeMotor2.setPower(0);
    }

    public void setTargetTPS(double targetTPS) {
        target = clampTarget(targetTPS);
    }

    public void startOuttaking(double targetTPS) {
        target = clampTarget(targetTPS);
    }

    public void linearRegression(double distanceCM) {
        distanceToGoal = distanceCM;
        target = clampTarget((regressionSlope * distanceCM) + regressionIntercept);
    }

    public void linearRegression(double formula, double distanceCM, double yIntercept) {
        distanceToGoal = distanceCM;
        target = clampTarget((formula * distanceCM) + yIntercept);
    }

    public void stopOuttake() {
        target = 0;
        lastOutput = 0;

        if (outtakeMotor1 != null) {
            outtakeMotor1.setPower(0);
        }

        if (outtakeMotor2 != null) {
            outtakeMotor2.setPower(0);
        }
    }

    public void updatePIDF() {
        controller.setPIDF(P, I, D, F);

        leftVelocity = Math.abs(outtakeMotor1.getVelocity());
        rightVelocity = Math.abs(outtakeMotor2.getVelocity());
        currentTPS = (leftVelocity + rightVelocity) / 2.0;

        if (target <= 0) {
            lastOutput = 0;
            outtakeMotor1.setPower(0);
            outtakeMotor2.setPower(0);
            return;
        }

        double output = controller.calculate(currentTPS, target);
        output = clampPower(output);

        lastOutput = output;

        outtakeMotor1.setPower(output);
        outtakeMotor2.setPower(output);
    }

    public boolean isAtSpeed(double threshold) {
        return target > 0 && currentTPS >= target * threshold;
    }

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

    private double clampTarget(double value) {
        return Math.max(MIN_TARGET_TPS, Math.min(MAX_TARGET_TPS, value));
    }

    private double clampPower(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }
}