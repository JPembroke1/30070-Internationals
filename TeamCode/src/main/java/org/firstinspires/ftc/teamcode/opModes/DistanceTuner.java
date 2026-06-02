package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="FlywheelTunerWithDistance", group="Tuning")
public class DistanceTuner extends OpMode {

    // Drive motors
    private DcMotor rightBack;
    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor leftBack;

    // Flywheel motors
    private DcMotorEx Shooter;
    private DcMotorEx Shooter2;

    private DcMotor Intake;
    private DcMotor Intake2;

    // Pedro Pathing
    private Follower follower;
    private Pose curPose;
    private final Pose startPose = new Pose(51, 91, Math.toRadians(143));

    // Goal position coordinates
    private static final double GOAL_X = 13.0;
    private static final double GOAL_Y = 135.0;

    // Tuning variables
    private int increment = 1;
    private double flywheelVelocity = 1130;

    // Button debouncing
    private boolean aPressed = false;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    @Override
    public void init() {
        // Initialize Pedro Pathing
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        // Initialize drive motors
        rightBack = hardwareMap.get(DcMotor.class, "backRight");
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBack = hardwareMap.get(DcMotor.class, "backLeft");

        // Set drive motor directions
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        // Initialize flywheel motors
        Shooter = hardwareMap.get(DcMotorEx.class, "outtakeLeft");
        Shooter2 = hardwareMap.get(DcMotorEx.class, "outtakeRight");
        Intake = hardwareMap.get(DcMotor.class, "intakeMotorFront");
        Intake2 = hardwareMap.get(DcMotor.class, "intakeMotorBack");

        // Set motor directions
        Shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        Shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        //Intake2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motor modes
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Configure PIDF coefficients
        PIDFCoefficients SHOOTERpidfCoefficients = new PIDFCoefficients(0.002, 0, 0, 15.5);
        Shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SHOOTERpidfCoefficients);
        Shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SHOOTERpidfCoefficients);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Goal Position", "X: %.1f, Y: %.1f", GOAL_X, GOAL_Y);
        telemetry.addData("Controls", "A: Cycle Increment | DPad Up/Down: Velocity");
    }

    @Override
    public void loop() {
        // Update Pedro Pathing localization
        follower.update();
        curPose = follower.getPose();

        // Get current robot coordinates
        double robotX = curPose.getX();
        double robotY = curPose.getY();

        // Calculate distance to goal
        double deltaX = GOAL_X - robotX;
        double deltaY = GOAL_Y - robotY;
        double distanceToGoal = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        // DRIVE CONTROLS
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x * 0.8;

        double leftFrontPower = drive + strafe + turn;
        double rightFrontPower = drive - strafe - turn;
        double leftBackPower = drive - strafe + turn;
        double rightBackPower = drive + strafe - turn;

        double maxPower = Math.max(Math.abs(leftFrontPower),
                Math.max(Math.abs(rightFrontPower),
                        Math.max(Math.abs(leftBackPower),
                                Math.abs(rightBackPower))));

        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightBackPower /= maxPower;
        }

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);

        // Run Intake Motors
        Intake2.setPower(1);
        Intake.setPower(1);

        // A button - Cycle increment
        if (gamepad1.a && !aPressed) {
            aPressed = true;
            if (increment == 1) increment = 5;
            else if (increment == 5) increment = 10;
            else increment = 1;
        } else if (!gamepad1.a) {
            aPressed = false;
        }

        // Increase velocity
        if (gamepad1.dpad_up && !dpadUpPressed) {
            dpadUpPressed = true;
            flywheelVelocity += increment;
        } else if (!gamepad1.dpad_up) {
            dpadUpPressed = false;
        }

        // Decrease velocity
        if (gamepad1.dpad_down && !dpadDownPressed) {
            dpadDownPressed = true;
            flywheelVelocity -= increment;
            if (flywheelVelocity < 0) flywheelVelocity = 0;
        } else if (!gamepad1.dpad_down) {
            dpadDownPressed = false;
        }

        // Apply velocity
        Shooter.setVelocity(flywheelVelocity);
        Shooter2.setVelocity(flywheelVelocity);

        // Telemetry
        telemetry.addData("=== POSITION DATA ===", "");
        telemetry.addData("Robot Position", "X: %.1f, Y: %.1f", robotX, robotY);
        telemetry.addData("Distance to Goal", "%.2f inches", distanceToGoal);
        telemetry.addData("", "");
        telemetry.addData("=== TUNING VALUES ===", "");
        telemetry.addData("Current Increment", increment);
        telemetry.addData("Flywheel Velocity", "%.0f", flywheelVelocity);
        telemetry.addData("", "");
        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("Left Stick", "Drive");
        telemetry.addData("Right Stick", "Turn");
        telemetry.addData("A Button", "Cycle Increment (1/5/10)");
        telemetry.addData("DPad Up/Down", "Adjust Velocity");
        telemetry.update();
    }
}