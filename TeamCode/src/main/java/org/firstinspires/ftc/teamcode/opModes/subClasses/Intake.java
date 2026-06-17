package org.firstinspires.ftc.teamcode.opModes.subClasses;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Intake {

    public DcMotor intakeMotorFront;
    public DcMotor intakeMotorBack;
    public DigitalChannel ballSensor;
    public Servo led;

    public static double red = 0.277;
    public static double green = 0.5;
    public static double detectTimeSeconds = 0.2;

    private long detectionStartTime = -1;

    public void init(@NonNull HardwareMap hardwareMap) {
        intakeMotorFront = hardwareMap.get(DcMotor.class, "intakeMotorFront");
        intakeMotorBack = hardwareMap.get(DcMotor.class, "intakeMotorBack");

        intakeMotorFront.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);

        ballSensor = hardwareMap.get(DigitalChannel.class, "ballSensor");
        ballSensor.setMode(DigitalChannel.Mode.INPUT);

        led = hardwareMap.get(Servo.class, "led");
        led.setPosition(red);
    }

    public void update() {
        boolean ballDetected = isBallDetected();

        if (ballDetected) {
            if (detectionStartTime < 0) {
                detectionStartTime = System.currentTimeMillis();
            }

            double detectedTime = (System.currentTimeMillis() - detectionStartTime) / 1000.0;

            if (detectedTime >= detectTimeSeconds) {
                led.setPosition(green);
            } else {
                led.setPosition(red);
            }
        } else {
            detectionStartTime = -1;
            led.setPosition(red);
        }
    }

    public boolean isBallDetected() {
        return ballSensor.getState();
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