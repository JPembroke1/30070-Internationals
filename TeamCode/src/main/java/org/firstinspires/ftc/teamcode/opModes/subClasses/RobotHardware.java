package org.firstinspires.ftc.teamcode.opModes.subClasses;

import android.os.DropBoxManager;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Intake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Outtake;

public class RobotHardware {

    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor intakeMotorFront = null;
    public DcMotor intakeMotorBack = null;


    public Servo hoodServo = null;
    public Servo blockServo = null;

    public RobotHardware(HardwareMap hardwareMap) {
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        blockServo = hardwareMap.get(Servo.class, "blockServo");

        blockServo.setDirection(Servo.Direction.REVERSE);
    }


    public void close_range() {
        hoodServo.setPosition(RobotSettings.close);
    }

    public void mid_range() {
        hoodServo.setPosition(RobotSettings.mid);
    }

    public void long_range() {
        hoodServo.setPosition(RobotSettings.far);
    }

    public void linearHoodRegression(double formula, double distance, double yIntercept) {
        hoodServo.setPosition(formula * distance + yIntercept);
    }

    public void block() {
        blockServo.setPosition(RobotSettings.block);
    }

    public void release() {
        blockServo.setPosition(RobotSettings.release);
    }

    public void reset_all() {
        hoodServo.setPosition(RobotSettings.close);
        blockServo.setPosition(RobotSettings.block);
    }


}
