package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opModes.subClasses.Intake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Outtake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.RobotHardware;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Configurable
@TeleOp(name = "InternationalTelopsOfTheAwesomeness", group = "Linear OpMode")
public class InternationalsOfTheTeleops extends LinearOpMode {

    public static double formulaResult = 1.75;
    public static double yIntercept = 633.333333;

    public static double distance = 0;

    public static boolean regressionEnabled = false;

    public static double GOAL_X = 0;
    public static double GOAL_Y = 138;

    public static double RESET_POSE_X = 21;
    public static double RESET_POSE_Y = 121;
    public static double RESET_POSE_HEADING_DEG = 143;

    public static double offsetX = 0;
    public static double offsetY = 0;

    public static boolean MANUAL_TURRET_TEST = false;
    public static double MANUAL_TURRET_SERVO_POS = 0.5;

    public static boolean ENABLE_LIVE_POSE = true;

    public static double HOOD_RESET_POSITION = 0.16;

    private RobotHardware robotHardware;
    private Intake intake;
    private Outtake outtake;
    private Turret turret;
    private Follower follower;

    private List<LynxModule> hubs;

    private Pose livePose;
    private boolean liveTrackingArmed = false;

    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;

    private final ElapsedTime runtime = new ElapsedTime();

    private boolean prevDpadRight = false;
    private boolean prevDpadUp = false;

    private boolean dpadRightPressed = false;
    private boolean dpadUpPressed = false;

    private boolean aiming = false;

    private double axial = 0;
    private double lateral = 0;
    private double yaw = 0;
    private double frontLeftPower = 0;
    private double frontRightPower = 0;
    private double backLeftPower = 0;
    private double backRightPower = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initDrive();
        initSubsystems();
        initPoseState();
        initFollower();

        turret.setPose(new Pose(GOAL_X + offsetX, GOAL_Y + offsetY, 0));

        telemetry.addLine("Init complete");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        runtime.reset();

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }

            updateButtonEdges();
            handlePoseSeedAndLiveTracking();
            updateLivePose();

            turret.setPose(new Pose(GOAL_X + offsetX, GOAL_Y + offsetY, 0));

            intake.update();

            handleDrive();
            handleRegressionButtons();
            handleIntake();
            handleTurret();
            updateRegressionShotLogic();

            outtake.updatePIDF();

            sendTelemetry();
        }
    }

    private void initDrive() {
        telemetry.addLine("Init: drive");
        telemetry.update();

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void initSubsystems() {
        telemetry.addLine("Init: subsystems");
        telemetry.update();

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        robotHardware = new RobotHardware(hardwareMap);

        outtake = new Outtake();
        outtake.init(hardwareMap);

        intake = new Intake();
        intake.init(hardwareMap);

        turret = new Turret();
        turret.init(hardwareMap);
    }

    private void initPoseState() {
        Pose storedPose = PoseStorage.getPose();

        if (storedPose != null) {
            livePose = storedPose;
        } else {
            livePose = new Pose(
                    RESET_POSE_X,
                    RESET_POSE_Y,
                    Math.toRadians(RESET_POSE_HEADING_DEG)
            );
            PoseStorage.setPose(livePose);
        }
    }

    private void initFollower() {
        telemetry.addLine("Init: follower");
        telemetry.update();

        try {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(livePose);
            telemetry.addData("Pedro", "OK");
        } catch (Exception e) {
            follower = null;
            telemetry.addData("Pedro failed", e.getClass().getSimpleName());
        }

        telemetry.update();
    }

    private void updateButtonEdges() {
        dpadRightPressed = gamepad1.dpad_right && !prevDpadRight;
        dpadUpPressed = gamepad1.dpad_up && !prevDpadUp;

        prevDpadRight = gamepad1.dpad_right;
        prevDpadUp = gamepad1.dpad_up;
    }

    private void handlePoseSeedAndLiveTracking() {
        if (!dpadRightPressed) return;

        Pose resetPose = new Pose(
                RESET_POSE_X,
                RESET_POSE_Y,
                Math.toRadians(RESET_POSE_HEADING_DEG)
        );

        livePose = resetPose;
        PoseStorage.setPose(resetPose);

        if (follower != null) {
            try {
                follower.setPose(resetPose);
            } catch (Exception e) {
                telemetry.addData("Pose seed error", e.getClass().getSimpleName());
            }
        }

        liveTrackingArmed = true;
    }

    private void updateLivePose() {
        if (!ENABLE_LIVE_POSE) return;
        if (!liveTrackingArmed) return;
        if (follower == null) return;

        try {
            follower.update();
            livePose = follower.getPose();
            PoseStorage.setPose(livePose);
        } catch (Exception e) {
            telemetry.addData("Pedro update error", e.getClass().getSimpleName());
        }
    }

    private void handleDrive() {
        axial = applyDeadband(-gamepad1.left_stick_y, 0.08);
        lateral = applyDeadband(gamepad1.left_stick_x, 0.08);
        yaw = applyDeadband(gamepad1.right_stick_x, 0.08) * 0.9;

        frontLeftPower = axial + lateral + yaw;
        frontRightPower = axial - lateral - yaw;
        backLeftPower = axial - lateral + yaw;
        backRightPower = axial + lateral - yaw;

        double max = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );

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
    }

    private void handleRegressionButtons() {
        if (gamepad1.x) {
            regressionEnabled = true;

        } else if (gamepad1.y) {
            if (regressionEnabled) {
                regressionEnabled = false;
                outtake.stopOuttake();
                robotHardware.block();
            } else {
                robotHardware.setHoodPosition(HOOD_RESET_POSITION);
            }
        }
    }

    private void handleIntake() {
        if (gamepad1.right_bumper) {
            robotHardware.block();
            intake.intake(1.0, 1.0);

        } else if (gamepad1.right_trigger > 0.6) {
            if (regressionEnabled) {
                robotHardware.release();
                intake.intake(0.7, 0.9);
            } else {
                robotHardware.block();
                intake.intakeStop();
            }

        } else {
            robotHardware.block();
            intake.intakeStop();
        }
    }

    private void handleTurret() {
        if (dpadUpPressed) {
            aiming = !aiming;
        }

        if (MANUAL_TURRET_TEST) {
            MANUAL_TURRET_SERVO_POS = Math.max(0.0, Math.min(1.0, MANUAL_TURRET_SERVO_POS));
            turret.rotationalTurretServo.setPosition(MANUAL_TURRET_SERVO_POS);
            return;
        }

        if (aiming && livePose != null) {
            try {
                turret.aimTurret(livePose);
            } catch (Exception e) {
                telemetry.addData("Turret aim error", e.getClass().getSimpleName());
            }
        } else {
            turret.centre();
        }
    }

    private void updateRegressionShotLogic() {
        double robotX = livePose != null ? livePose.getX() : 0;
        double robotY = livePose != null ? livePose.getY() : 0;

        double goalX = GOAL_X + offsetX;
        double goalY = GOAL_Y + offsetY;

        distance = Math.hypot(goalX - robotX, goalY - robotY);

        if (regressionEnabled) {
            double distanceCM = distance * 2.54;
            outtake.linearRegression(formulaResult, distanceCM, yIntercept);
            robotHardware.linearHoodRegression(distanceCM);
        }
    }

    private void sendTelemetry() {
        double robotX = livePose != null ? livePose.getX() : 0;
        double robotY = livePose != null ? livePose.getY() : 0;
        double robotHeadingDeg = livePose != null ? Math.toDegrees(livePose.getHeading()) : 0;

        double goalX = GOAL_X + offsetX;
        double goalY = GOAL_Y + offsetY;
        double dxGoal = goalX - robotX;
        double dyGoal = goalY - robotY;
        double angleToGoalDeg = Math.toDegrees(Math.atan2(dyGoal, dxGoal));

        telemetry.addData("Status", "Running - %.1fs", runtime.seconds());

        telemetry.addLine("--- Ball Sensor ---");
        telemetry.addData("Ball Detected", intake.isBallDetected() ? "YES" : "NO");

        telemetry.addLine("--- Field Pose ---");
        telemetry.addData("Field X", "%.2f", robotX);
        telemetry.addData("Field Y", "%.2f", robotY);
        telemetry.addData("Heading", "%.1f deg", robotHeadingDeg);
        telemetry.addData("Live Tracking Armed", liveTrackingArmed);

        telemetry.addLine("--- Goal ---");
        telemetry.addData("Goal X", "%.2f", goalX);
        telemetry.addData("Goal Y", "%.2f", goalY);
        telemetry.addData("Distance (in)", "%.1f", distance);
        telemetry.addData("Angle To Goal", "%.1f deg", angleToGoalDeg);

        telemetry.addLine("--- Turret ---");
        telemetry.addData("Aiming", aiming);
        telemetry.addData("Relative Angle", "%.1f deg", Turret.relativeAngleDeg);
        telemetry.addData("Desired Servo", "%.3f", Turret.desiredServo);

        telemetry.addLine("--- Shooter ---");
        telemetry.addData("Regression", regressionEnabled ? "ON" : "OFF");
        telemetry.addData("Distance (cm)", "%.1f", distance * 2.54);
        telemetry.addData("Target TPS", "%.0f", Outtake.target);
        telemetry.addData("Current TPS", "%.0f", Outtake.currentTPS);

        telemetry.addLine("--- Hood ---");
        telemetry.addData("Last Hood Cmd", "%.3f", RobotHardware.lastCommandedHood);
        telemetry.addData("Filtered Dist CM", "%.2f", RobotHardware.filteredDistanceCM);
        telemetry.addData("Hood Initialised", RobotHardware.hoodInitialised);
        telemetry.addData("Hood Reset Pos", "%.3f", HOOD_RESET_POSITION);

        telemetry.addLine("--- Drive ---");
        telemetry.addData("Axial", "%.3f", axial);
        telemetry.addData("Lateral", "%.3f", lateral);
        telemetry.addData("Yaw", "%.3f", yaw);
        telemetry.addData("FL", "%.3f", frontLeftPower);
        telemetry.addData("FR", "%.3f", frontRightPower);
        telemetry.addData("BL", "%.3f", backLeftPower);
        telemetry.addData("BR", "%.3f", backRightPower);

        telemetry.addLine("--- Controls ---");
        telemetry.addData("G1 X", "Regression ON");
        telemetry.addData("G1 Y", "Regression OFF or hood reset");
        telemetry.addData("G1 Dpad Right", "Reset pose");
        telemetry.addData("G1 Dpad Up", "Toggle aiming");
        telemetry.addData("G1 RB", "Intake with block closed");
        telemetry.addData("G1 RT", "Feed if regression on");

        telemetry.update();
    }

    private double applyDeadband(double value, double deadband) {
        return Math.abs(value) > deadband ? value : 0.0;
    }
}