package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "Hood Servo Test", group = "Tests")
public class HoodServoTest extends OpMode {

    public static double manualPosition = 0.5;
    public static boolean reversed = true;

    private Servo hoodServo;

    @Override
    public void init() {
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        applyDirection();
        hoodServo.setPosition(manualPosition);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            manualPosition = 0.0;
        }
        if (gamepad1.b) {
            manualPosition = 0.5;
        }
        if (gamepad1.y) {
            manualPosition = 1.0;
        }

        if (gamepad1.dpad_up) {
            manualPosition += 0.01;
        }
        if (gamepad1.dpad_down) {
            manualPosition -= 0.01;
        }

        if (gamepad1.x) {
            reversed = true;
        }
        if (gamepad1.left_stick_button) {
            reversed = false;
        }

        manualPosition = Math.max(0.0, Math.min(1.0, manualPosition));

        applyDirection();
        hoodServo.setPosition(manualPosition);

        telemetry.addLine("Hood Servo Test");
        telemetry.addData("Commanded Position", "%.3f", manualPosition);
        telemetry.addData("Direction", reversed ? "REVERSE" : "FORWARD");
        telemetry.addLine("A=0.0  B=0.5  Y=1.0");
        telemetry.addLine("DPAD UP/DOWN = fine adjust");
        telemetry.addLine("X = reverse, Left Stick Button = forward");
        telemetry.update();
    }

    private void applyDirection() {
        hoodServo.setDirection(reversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
    }
}