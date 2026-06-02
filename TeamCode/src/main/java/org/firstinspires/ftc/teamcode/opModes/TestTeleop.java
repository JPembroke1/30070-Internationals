package org.firstinspires.ftc.teamcode.opModes;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Configurable
@TeleOp(name="NOTTHISONE", group="Linear OpMode")
public class TestTeleop extends LinearOpMode {

    /* ---------------- SHOOTER ---------------- */

    private static double TARGET_SPEED = 0;
    public static double target = 0;

    public static double P = 0.0024;
    public static double I = 0.0;
    public static double D = 0.0001;

    public PIDController controller;

    /* ---------------- TURRET ---------------- */

    public static double turretCenter = 0.52;
    public static double minServo = 0.08;
    public static double maxServo = 0.92;

    // turret gear ratio 2:1 → servo moves twice turret
    public static double maxTurretAngle = Math.toRadians(45);

    private static double servoDir = 0.5;
    boolean autoAim = false;

    public static Pose goalPose = new Pose(0, 144, 0); // cm

    /* ---------------- HARDWARE ---------------- */

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    private DcMotor intakeMotorFront = null;
    private DcMotor intakeMotorBack = null;

    private Servo rotationalTurretServo = null;
    private Servo hoodServo = null;

    private DcMotorEx outtakeMotor1 = null;
    private DcMotorEx outtakeMotor2 = null;

    /* ---------------- PINPOINT ---------------- */

    GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {

        /* ---------------- HARDWARE INIT ---------------- */

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");

        intakeMotorFront = hardwareMap.get(DcMotor.class, "intakeMotorFront");
        intakeMotorBack = hardwareMap.get(DcMotor.class, "intakeMotorBack");

        rotationalTurretServo = hardwareMap.get(Servo.class, "rotationalTurretServo");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        outtakeMotor1 = hardwareMap.get(DcMotorEx.class, "outtakeLeft");
        outtakeMotor2 = hardwareMap.get(DcMotorEx.class, "outtakeRight");

        /* ---------------- PINPOINT INIT ---------------- */

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0));

        /* ---------------- MOTOR DIRECTIONS ---------------- */

        outtakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotorFront.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotationalTurretServo.setPosition(servoDir);

        controller = new PIDController(P, I, D);

        telemetry.addData("Status","Initialized");
        telemetry.update();

        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {

            /* ---------------- PINPOINT UPDATE ---------------- */

            pinpoint.update();

            double robotX = pinpoint.getPosX(DistanceUnit.CM);
            double robotY = pinpoint.getPosY(DistanceUnit.CM);
            double robotHeading = Math.toRadians(pinpoint.getHeading(AngleUnit.DEGREES));

            /* ---------------- AUTO TURRET ---------------- */

            if(autoAim){

                double dx = goalPose.getX() - robotX;
                double dy = goalPose.getY() - robotY;

                double targetHeading = Math.atan2(dy, dx);

                double turretAngle = targetHeading - robotHeading;

                while(turretAngle > Math.PI) turretAngle -= 2*Math.PI;
                while(turretAngle < -Math.PI) turretAngle += 2*Math.PI;

                // clamp to max turret movement
                turretAngle = Math.max(-maxTurretAngle, Math.min(maxTurretAngle, turretAngle));

                // Map turretAngle to servo position
                double servoRange = maxServo - minServo;
                double normalized = turretAngle / maxTurretAngle;
                normalized = Math.max(-1, Math.min(1, normalized));
                double servoPos = turretCenter + normalized * (servoRange / 2);
                servoPos = Math.max(minServo, Math.min(maxServo, servoPos));

                rotationalTurretServo.setPosition(servoPos);

                // Debug telemetry
                telemetry.addData("dx", dx);
                telemetry.addData("dy", dy);
                telemetry.addData("targetHeading (deg)", Math.toDegrees(targetHeading));
                telemetry.addData("turretAngle (deg)", Math.toDegrees(turretAngle));
                telemetry.addData("servoPos", servoPos);
            }

            if (gamepad1.dpad_left) {
                goalPose = new Pose(robotX, robotY, robotHeading);
            }

            /* ---------------- SHOOTER PID ---------------- */

            updatePIDF();

            /* ---------------- DRIVE ---------------- */

            double axial   = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw     = gamepad1.right_stick_x * 0.7;

            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            /* ---------------- INTAKE ---------------- */

            if(gamepad1.right_bumper){
                intakeMotorFront.setPower(0.8);
                intakeMotorBack.setPower(0.8);
            } else {
                intakeMotorFront.setPower(0);
                intakeMotorBack.setPower(0);
            }

            /* ---------------- SHOOTER BUTTONS ---------------- */

            if(gamepad1.cross){
                target = TARGET_SPEED;
            }

            if(gamepad1.triangle){
                target = 0;
            }

            /* ---------------- TURRET CONTROLS ---------------- */

            if(gamepad1.dpad_up){
                autoAim = true;
            }

            if(gamepad1.dpad_down){
                autoAim = false;
            }

            if(!autoAim){
                if(gamepad1.square) servoDir += 0.01;
                if(gamepad1.circle) servoDir -= 0.01;
                servoDir = Math.max(0,Math.min(1,servoDir));
                rotationalTurretServo.setPosition(servoDir);
            }

            /* ---------------- TELEMETRY ---------------- */

            telemetry.addData("AutoAim",autoAim);
            telemetry.addData("RobotX",robotX);
            telemetry.addData("RobotY",robotY);
            telemetry.addData("Heading (deg)", Math.toDegrees(robotHeading));
            telemetry.addData("TurretServo",rotationalTurretServo.getPosition());
            telemetry.update();

        }

    }

    private void updatePIDF(){

        controller.setPID(P,I,D);

        if(target == 0){
            outtakeMotor1.setPower(0);
            outtakeMotor2.setPower(0);
            return;
        }

        double currentTPS = outtakeMotor1.getVelocity();
        double response = controller.calculate(currentTPS,target);
        response = Math.max(-1,Math.min(1,response));

        outtakeMotor1.setPower(response);
        outtakeMotor2.setPower(response);

    }
}