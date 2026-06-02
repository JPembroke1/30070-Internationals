package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.opModes.InternationalsOfTheTeleops.D;
import static org.firstinspires.ftc.teamcode.opModes.InternationalsOfTheTeleops.I;
import static org.firstinspires.ftc.teamcode.opModes.InternationalsOfTheTeleops.P;
import static org.firstinspires.ftc.teamcode.opModes.InternationalsOfTheTeleops.F;
import static org.firstinspires.ftc.teamcode.opModes.InternationalsOfTheTeleops.currentTPS;


import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "blue side goal")
public class BlueAuto extends OpMode {

    ElapsedTime timer = new ElapsedTime();

    // --- Motors ---
    private DcMotorEx outtakeMotor1, outtakeMotor2;

    DcMotor intakeMotorFront;
    DcMotor intakeMotorBack;
    Servo blockServo;
    Servo hoodServo;
    Servo rotationalTurretServo;

    public PIDFController controller;
    private Follower follower;
    private PathChain path1;
    private PathChain path2;

    private double target = 1100;

    private int pathState = 0;

    public void init() {
        controller = new PIDFController(P, I, D, F);

        intakeMotorFront = hardwareMap.dcMotor.get("intakeMotorFront");
        intakeMotorBack = hardwareMap.dcMotor.get("intakeMotorBack");
        blockServo = hardwareMap.get(Servo.class, "blockServo");

        blockServo.setPosition(0.2);

        outtakeMotor1 = hardwareMap.get(DcMotorEx.class, "outtakeLeft");
        outtakeMotor2 = hardwareMap.get(DcMotorEx.class, "outtakeRight");

        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        blockServo = hardwareMap.get(Servo.class, "blockServo");

        rotationalTurretServo = hardwareMap.get(Servo.class, "rotationalTurretServo");

        outtakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor1.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotorFront.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);

        this.follower = Constants.createFollower(hardwareMap);
        this.path1 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(72, 8),
                                new Pose(72.000, 72.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                .build();
        this.path2 = new PathBuilder(follower)
                .addPath(
                        new BezierLine(
                                new Pose(72.000, 72.000),
                                new Pose(38.000, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public void loop() {
        switch (pathState) {
            case 0:
                this.follower.followPath(path1);
                pathState += 1;
                break;
            case 1:
                this.follower.followPath(path2);
                pathState += 1;
                break;

        }
        follower.update();
    }

    private void updatePIDF() {

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
