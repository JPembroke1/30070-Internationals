package org.firstinspires.ftc.teamcode.opModes.subClasses;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.opModes.InternationalsOfTheTeleops;

public class Outtake {


    public DcMotorEx outtakeMotor1;
    public DcMotorEx outtakeMotor2;
    public PIDFController controller;

    public static double target = 0;
    public static double currentTPS = 0;
    public static double P = 20;
    public static double I = 0.0;
    public static double D = 0;
    public static double F = 20;


    RobotHardware robotHardware;

    public void init(@NonNull HardwareMap hardwareMap) {
        outtakeMotor1 = hardwareMap.get(DcMotorEx.class, "outtakeLeft");
        outtakeMotor2 = hardwareMap.get(DcMotorEx.class, "outtakeRight");

        outtakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor1.setDirection(DcMotorSimple.Direction.FORWARD);

        controller = new PIDFController(P, I, D, F);
        robotHardware = new RobotHardware(hardwareMap);

    }

    public void startOuttaking(double outtakePower) {

        target = outtakePower;

    }

    public void linearRegression(double formula, double distance, double yIntercept) {

        target = (formula * distance + yIntercept); //LINEAR REGRESSION
    }

    public void stopOuttake() {
        target = 0;
    }

    public void updatePIDF() {

        if (target == 0) {
            outtakeMotor1.setPower(0);
            outtakeMotor2.setPower(0);
            return;
        }

        currentTPS = (outtakeMotor1.getVelocity() + outtakeMotor2.getVelocity()) / 2;

        if (outtakeMotor1.getVelocity() < 100) {
            currentTPS = outtakeMotor2.getVelocity();
        } else if (outtakeMotor2.getVelocity() < 100) {
            currentTPS = outtakeMotor1.getVelocity();
        } else if (outtakeMotor1.getVelocity() < 100 && outtakeMotor2.getVelocity() < 100 && InternationalsOfTheTeleops.ActivatedShoot) {
            outtakeMotor1.setPower(0.5);
            outtakeMotor2.setPower(0.5);
        } else {
            currentTPS = (outtakeMotor1.getVelocity() + outtakeMotor2.getVelocity()) / 2;
        }

        double error = target - currentTPS;

        // Deadband
        if (Math.abs(error) < 20) {
            error = 0;
        }

        // Feedforward
        double ff = target * F;

        // Simple proportional correction
        double pid = error * P;

        double response = ff + pid;

        // Overspeed cap
        if (currentTPS > target) {
            response = Math.min(response, ff);
        }

        // Clamp motor power
        response = Math.max(0, Math.min(1, response));

        outtakeMotor1.setPower(response);
        outtakeMotor2.setPower(response);
    }

}
