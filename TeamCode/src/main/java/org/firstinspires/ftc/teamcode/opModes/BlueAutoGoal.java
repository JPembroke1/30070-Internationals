/*package org.firstinspires.ftc.teamcode.opModes;

import static org.firstinspires.ftc.teamcode.opModes.InternationalsOfTheTeleops.D;
import static org.firstinspires.ftc.teamcode.opModes.InternationalsOfTheTeleops.I;
import static org.firstinspires.ftc.teamcode.opModes.InternationalsOfTheTeleops.P;

import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous(name = "blue side goal")
public class BlueAutoGoal extends OpMode {

    ElapsedTime timer = new ElapsedTime();

    // --- Motors ---
    private DcMotorEx outtake, outtake1;

    DcMotor intakeMotor;
    Servo blockServo;
    Servo blockServo2;
    CRServo intakeServo1;
    CRServo intakeServo2;

    public PIDController controller;
    private Follower follower;
    private PathChain path1;

    private PathChain path2;

    private double target = 1100;

    private int pathState = 0;

    public void init() {
        controller = new PIDController(P, I, D);

            intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
            blockServo = hardwareMap.get(Servo.class, "rightServo");
            blockServo2 = hardwareMap.get(Servo.class, "blockServo2");

            blockServo2.setDirection(Servo.Direction.REVERSE);

            intakeServo1 = hardwareMap.get(CRServo.class, "intakeServo1");
            intakeServo2 = hardwareMap.get(CRServo.class, "intakeServo2");
            intakeServo2.setDirection(CRServo.Direction.REVERSE);

            blockServo.setPosition(0.5);
            blockServo2.setPosition(0.5);

            outtake = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
            outtake1 = hardwareMap.get(DcMotorEx.class, "outtakeMotor2");
            // Configure encoder motor
            outtake1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            outtake1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            // Secondary motor follows
            outtake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            outtake.setDirection(DcMotorSimple.Direction.REVERSE);
            outtake1.setDirection(DcMotorSimple.Direction.REVERSE);

        this.follower = Constants.createFollower(hardwareMap);
        this.path1 = new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(0, 0),
                        new Pose(-5, 0, 0)
                ))
                .setConstantHeadingInterpolation(0)
                .build();

        this.path2 = new PathBuilder(follower)
                .addPath(new BezierLine(
                        new Pose(-5, 0),
                        new Pose(-8, 40)
                ))
                .setConstantHeadingInterpolation(-0.865)
                .build();
    }

    public void loop() {
        switch (pathState) {
            case 0:
                this.follower.followPath(path1);
                pathState += 1;
                break;
            case 1:
                if (!this.follower.isBusy()) {
                    pathState += 1;
                    this.follower.breakFollowing();
                }
                break;
            case 2:
                timer.reset();
                target = 1100;
                while (outtake1.getVelocity() < 800) {
                    updatePIDF();
                }
                timer.reset();
                while (timer.seconds() < 5) {
                    updatePIDF();

                    blockServo.setPosition(0.25);
                    blockServo2.setPosition(0.25);
                    intakeMotor.setPower(1);
                    intakeServo1.setPower(0.7);
                    intakeServo2.setPower(0.7);
                }
                this.outtake.setPower(0);
                this.outtake1.setPower(0);
                pathState += 1;

                // Shoot
                break;
            case 3:
                blockServo.setPosition(0);
                blockServo2.setPosition(0);
                intakeMotor.setPower(0);
                intakeServo1.setPower(0);
                intakeServo2.setPower(0);
                pathState += 1;
                break;
            case 4:
                follower.followPath(path2);
                break;
        }

        follower.update();
    }

    private void updatePIDF() {
        if (target == 0) {
            outtake.setPower(0);
            outtake1.setPower(0);
            return;
        }
        double currentTPS = outtake1.getVelocity();
        double response = controller.calculate(currentTPS, target);
        outtake.setPower(response);
        outtake1.setPower(response);
    }
}
*/