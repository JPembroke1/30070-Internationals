package org.firstinspires.ftc.teamcode.opModes.subClasses;

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

    public static double green = 0.500;
    public static double off = 0.000;

    public static boolean rawSensorState = false;
    public static boolean sensorTriggered = false;

    public static double lastLedPosition = 0.0;
    public static double lastFrontPower = 0.0;
    public static double lastBackPower = 0.0;

    public void init(HardwareMap hardwareMap) {
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

        rawSensorState = ballSensor.getState();
        sensorTriggered = rawSensorState;

        lastLedPosition = off;
        lastFrontPower = 0.0;
        lastBackPower = 0.0;

        setLedOff();
        intakeStop();
    }

    public void update() {
        rawSensorState = ballSensor.getState();
        sensorTriggered = rawSensorState;

        if (sensorTriggered) {
            setLedGreen();
        } else {
            setLedOff();
        }
    }

    public boolean isSensorTriggered() {
        rawSensorState = ballSensor.getState();
        sensorTriggered = rawSensorState;
        return sensorTriggered;
    }

    public void setLedGreen() {
        setLedPosition(green);
    }

    public void setLedOff() {
        setLedPosition(off);
    }

    public void setLedPosition(double position) {
        double clippedPosition = clamp(position, 0.0, 1.0);

        led.setPosition(clippedPosition);
        lastLedPosition = clippedPosition;
    }

    public void intake(double powerFront, double powerBack) {
        double clippedFront = clamp(powerFront, -1.0, 1.0);
        double clippedBack = clamp(powerBack, -1.0, 1.0);

        intakeMotorFront.setPower(clippedFront);
        intakeMotorBack.setPower(clippedBack);

        lastFrontPower = clippedFront;
        lastBackPower = clippedBack;
    }

    public void setFeederPower(double power) {
        double clippedPower = clamp(power, -1.0, 1.0);
        intake(clippedPower, clippedPower);
    }

    public void setFarZoneFeederPower(double power) {
        setFeederPower(power);
    }

    public void intakeStop() {
        intakeMotorFront.setPower(0.0);
        intakeMotorBack.setPower(0.0);

        lastFrontPower = 0.0;
        lastBackPower = 0.0;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}