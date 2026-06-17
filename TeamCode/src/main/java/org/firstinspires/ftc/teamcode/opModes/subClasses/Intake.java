package org.firstinspires.ftc.teamcode.opModes.subClasses;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Configurable
public class Intake {

    public DcMotor intakeMotorFront;
    public DcMotor intakeMotorBack;
    public DigitalChannel ballSensor;
    public ServoImplEx led;

    public static double red = 0.300;
    public static double green = 0.500;
    public static double off = 0.000;

    public static double detectTimeSeconds = 0.2;

    public static boolean rawSensorState = false;
    public static boolean ballDetected = false;
    public static double detectedTimeSeconds = 0.0;
    public static double lastLedPosition = 0.0;

    private long detectionStartTime = -1;

    public void init(@NonNull HardwareMap hardwareMap) {
        intakeMotorFront = hardwareMap.get(DcMotor.class, "intakeMotorFront");
        intakeMotorBack = hardwareMap.get(DcMotor.class, "intakeMotorBack");

        intakeMotorFront.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ballSensor = hardwareMap.get(DigitalChannel.class, "ballSensor");
        ballSensor.setMode(DigitalChannel.Mode.INPUT);

        led = hardwareMap.get(ServoImplEx.class, "led");
        led.setPwmRange(new PwmControl.PwmRange(500, 2500));
        led.setPwmEnable();

        detectionStartTime = -1;
        detectedTimeSeconds = 0.0;
        rawSensorState = false;
        ballDetected = false;

        setLedRed();
        intakeStop();
    }

    public void update() {
        rawSensorState = ballSensor.getState();
        ballDetected = isBallDetected();

        if (ballDetected) {
            if (detectionStartTime < 0) {
                detectionStartTime = System.currentTimeMillis();
            }

            detectedTimeSeconds = (System.currentTimeMillis() - detectionStartTime) / 1000.0;

            if (detectedTimeSeconds >= detectTimeSeconds) {
                setLedGreen();
            } else {
                setLedRed();
            }
        } else {
            detectionStartTime = -1;
            detectedTimeSeconds = 0.0;
            setLedRed();
        }
    }

    public boolean isBallDetected() {
        return ballSensor.getState();
    }

    public boolean isBallReady() {
        return ballDetected && detectedTimeSeconds >= detectTimeSeconds;
    }

    public void setLedRed() {
        setLedPosition(red);
    }

    public void setLedGreen() {
        setLedPosition(green);
    }

    public void setLedOff() {
        setLedPosition(off);
    }

    public void setLedPosition(double position) {
        double clippedPosition = Math.max(0.0, Math.min(1.0, position));
        led.setPosition(clippedPosition);
        lastLedPosition = clippedPosition;
    }

    public void intake(double powerFront, double powerBack) {
        intakeMotorFront.setPower(powerFront);
        intakeMotorBack.setPower(powerBack);
    }

    public void intakeStop() {
        intakeMotorFront.setPower(0);
        intakeMotorBack.setPower(0);
    }

    public void resetDetectionTimer() {
        detectionStartTime = -1;
        detectedTimeSeconds = 0.0;
    }
}

