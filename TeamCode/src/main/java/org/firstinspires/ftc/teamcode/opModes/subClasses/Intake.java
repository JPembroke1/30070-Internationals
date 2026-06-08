package org.firstinspires.ftc.teamcode.opModes.subClasses;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.opModes.InternationalsOfTheTeleops;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake{

    public DcMotor intakeMotorFront;
    public DcMotor intakeMotorBack;

    public void init (@NonNull HardwareMap hardwareMap) {
        intakeMotorFront = hardwareMap.get(DcMotor.class, "intakeMotorFront");
        intakeMotorBack = hardwareMap.get(DcMotor.class, "intakeMotorBack");
        intakeMotorFront.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void intake(double powerFront, double powerBack) {
            intakeMotorFront.setPower(powerFront);
            intakeMotorBack.setPower(powerBack);
    }

    public void intakeStop() {
        intakeMotorFront.setPower(0);
        intakeMotorBack.setPower(0);
    }

}
