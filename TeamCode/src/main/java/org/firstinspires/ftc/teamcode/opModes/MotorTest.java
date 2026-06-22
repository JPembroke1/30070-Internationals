
package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Motor Test", group = "Test")
public class MotorTest extends OpMode {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        telemetry.addLine("Motor Test Ready");
        telemetry.addLine("Press buttons to run motors");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Stop everything first
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Gamepad controls
        if (gamepad1.a) {
            frontLeft.setPower(0.3);
            telemetry.addLine("Running FRONT LEFT");
        }
        if (gamepad1.b) {
            frontRight.setPower(0.3);
            telemetry.addLine("Running FRONT RIGHT");
        }
        if (gamepad1.x) {
            backLeft.setPower(0.3);
            telemetry.addLine("Running BACK LEFT");
        }
        if (gamepad1.y) {
            backRight.setPower(0.3);
            telemetry.addLine("Running BACK RIGHT");
        }

        telemetry.update();
    }
}
