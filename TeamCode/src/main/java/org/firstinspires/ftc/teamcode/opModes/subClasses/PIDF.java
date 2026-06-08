/* package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.opModes.InternationalsOfTheTeleops;

@Disabled
public class PIDF {

    Outtake outtake;

        DcMotorEx outtakeMotor1 = outtake.outtakeMotor1;
        DcMotorEx outtakeMotor2 = outtake.outtakeMotor2;

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
        } else if (outtakeMotor1.getVelocity() < 100 && outtakeMotor2.getVelocity() < 100 && ActivatedShoot) {
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
 */