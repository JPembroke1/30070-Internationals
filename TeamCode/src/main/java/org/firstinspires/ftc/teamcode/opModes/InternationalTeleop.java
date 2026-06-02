package org.firstinspires.ftc.teamcode.opModes;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Disabled
@TeleOp(name = "AutoAimTeleop")
public class InternationalTeleop extends OpMode {

    public static double offsetX = 0;
    public static double offsetY = 0;

    private DcMotorEx outtake, outtake1;
    // --- PIDF Coefficients (Dashboard) ---
    public static double hoodPosition = 0.3;
    public static double target = 0;
    public static double P = 0.0024;
    public static double I = 0.0;
    public static double D = 0.0001;

    public static double ledColour = 0;
    public static double F = 5.0;
    // --- Motor constants ---
    public static double TICKS_PER_REV = 28.0;

    public static double TARGET_SPEED_SHORT = 1150;

    public static double TARGET_SPEED_LONG = 1700;

    public static double TARGET_SPEED_MID = 1350;

    public static double MID_HOOD_POSITION = 0.2;
    private double targetVelocity = 0.0;// in ticks/sec

    private Follower follower;
    private TelemetryManager telemetryM;

    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    public static Pose goalPose = new Pose(0, 144, 0); // default, but now dynamically updated with A

    private boolean active = false;

    private boolean holding = false;

    private Pose holdpose = null;

    private final double kP_xy = 2.2;

    private Pose currentPose = null;


    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    Servo rotationalTurretServo;
    Servo hoodServo;
    DcMotor outtakeMotor1;
    DcMotor outtakeMotor2;
    Servo blockServo;
    DcMotor intakeMotorFront;
    DcMotor intakeMotorBack;

    Servo led;
    Servo pacman;


    public PIDController controller;

    @Override
    public void init() {
        //follower = Constants.createFollower(hardwareMap);
        //follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        controller = new PIDController(P, I, D);
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backRightMotor = hardwareMap.dcMotor.get("rightBack");

        //intakeMotorFront = hardwareMap.dcMotor.get("intakeMotorFront");
        //intakeMotorBack = hardwareMap.dcMotor.get("intakeMotorBack");
        //blockServo = hardwareMap.get(Servo.class, "blockServo");

        //hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        //hoodServo.setDirection(Servo.Direction.REVERSE);

        //blockServo.setPosition(0.5);

        //outtakeMotor1 = hardwareMap.get(DcMotorEx.class, "outtakeMotor1");
        //outtakeMotor2 = hardwareMap.get(DcMotorEx.class, "outtakeMotor2");

        //hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        // Configure encoder motor
        /*outtake1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        // Secondary motor follows
        outtake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        updatePIDF();

        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake1.setDirection(DcMotorSimple.Direction.REVERSE); */
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        //updatePIDF();

        //follower.update();
        telemetryM.update();

        // -----------------------------
        // Slow mode
        // -----------------------------

        double translationX = -gamepad1.left_stick_x * 1.1;
        double translationY = -gamepad1.left_stick_y;
        double driverRotation = -gamepad1.right_stick_x * 0.7;

        boolean aiming = gamepad1.left_bumper;

        // -----------------------------
        // AIMING MODE (only auto-rotate)
        // -----------------------------
        if (aiming) {

            /*Pose robotPose = follower.getPose();

            double dx = goalPose.getX() - robotPose.getX();
            double dy = goalPose.getY() - robotPose.getY();
            double targetHeading = Math.atan2(dy, dx);

            double currentHeading = robotPose.getHeading();
            double headingError = targetHeading - currentHeading;

            headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));

            double kP = 2.0;
            double rotation = headingError * kP;

            rotation = Math.max(-1, Math.min(1, rotation));

            follower.setTeleOpDrive(translationY, translationX, rotation, true); */

        } else {

            follower.setTeleOpDrive(translationY, translationX, driverRotation, true);
        }

        // -----------------------------
        // A-button: SET CURRENT POSITION AS GOAL
        // -----------------------------
        /*if (gamepad1.dpad_down) {
            Pose robotPose = follower.getPose();
            goalPose = new Pose(robotPose.getX() + offsetX, robotPose.getY() + offsetY, robotPose.getHeading());
        }


        telemetry.addData("TPS", outtake1.getVelocity());

        if (gamepad1.dpad_left && !holding) {
            holding = true;

            follower.holdPoint(currentPose);
        } else {

            currentPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());

            holding = false;


        } */

        // -----------------------------
        // Telemetry
        // -----------------------------
        //telemetryM.debug("Robot Pose", follower.getPose());
        telemetryM.debug("Goal Pose", goalPose);
        telemetryM.debug("Aiming", aiming);
        telemetryM.debug("Slow Mode", slowMode);

        telemetry.update();
    }
    /*private void updatePIDF() {
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

    private double[] fieldErrorToRobot(double errX, double errY, double headingRad) {


        double cos = Math.cos(-headingRad);
        double sin = Math.sin(-headingRad);

        double robotX = errX - errY;
        double robotY = errX + errY;

        return new double[]{robotX, robotY};
    }
     */
}
