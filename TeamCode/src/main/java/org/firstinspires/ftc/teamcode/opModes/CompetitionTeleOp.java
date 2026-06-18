package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opModes.subClasses.Intake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Outtake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.RobotHardware;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.Locale;

@Configurable
@TeleOp(name = "Competition TeleOp", group = "TeleOp")
public class CompetitionTeleOp extends OpMode {

    private Follower follower;
    private Intake intake;
    private Outtake outtake;
    private Turret turret;
    private RobotHardware robotHardware;

    private List<LynxModule> allHubs;

    public static double BLUE_GOAL_X = 6.0;
    public static double BLUE_GOAL_Y = 138.0;

    public static double RED_GOAL_X = 138.0;
    public static double RED_GOAL_Y = 138.0;

    public static double BLUE_RESET_X = 24.0;
    public static double BLUE_RESET_Y = 138.0;
    public static double BLUE_RESET_HEADING_DEG = 145.0;

    public static double RED_RESET_X = 120.0;
    public static double RED_RESET_Y = 138.0;
    public static double RED_RESET_HEADING_DEG = 35.0;

    public static double TELEOP_GOAL_X = 6.0;
    public static double TELEOP_GOAL_Y = 138.0;

    public static double TELEOP_RESET_X = 24.0;
    public static double TELEOP_RESET_Y = 138.0;
    public static double TELEOP_RESET_HEADING_DEG = 145.0;

    private String allianceLabel = "UNKNOWN";

    private boolean previousG1DpadUp = false;
    private boolean previousG1DpadRight = false;
    private boolean previousLeftBumper = false;

    private boolean previousG2DpadLeft = false;
    private boolean previousG2DpadRight = false;
    private boolean previousG2Square = false;

    private boolean previousSquare = false;
    private boolean previousTriangle = false;
    private boolean previousG2Triangle = false;

    public static boolean START_IN_AUTO_AIM = true;

    private boolean autoAimMode = true;

    public static double AIM_OFFSET_STEP = 0.005;
    public static double AIM_OFFSET_MIN = -0.20;
    public static double AIM_OFFSET_MAX = 0.20;

    public static double DRIVE_SPEED = 1.0;
    public static double INTAKE_TURN_MULTIPLIER = 0.45;
    public static double FEED_DRIVE_MULTIPLIER = 0.50;

    public static boolean USE_STATIC_TARGET_TPS = false;
    public static double STATIC_TARGET_TPS = 1500.0;

    private boolean shooterEnabled = false;

    public static double SHOOT_TRIGGER_THRESHOLD = 0.2;

    public static double SHOOT_INTAKE_LEFT_POWER = 0.5;
    public static double SHOOT_INTAKE_RIGHT_POWER = 0.5;

    public static double COLLECT_INTAKE_LEFT_POWER = 1.0;
    public static double COLLECT_INTAKE_RIGHT_POWER = 1.0;

    public static boolean USE_POSE_DELTA_VELOCITY = true;

    private Pose previousVelocityPose = null;
    private double previousVelocityTime = 0.0;

    private double estimatedFieldVelocityX = 0.0;
    private double estimatedFieldVelocityY = 0.0;
    private double estimatedFieldSpeed = 0.0;

    @Override
    public void init() {
        configureBulkCaching();
        clearBulkCache();

        applyAllianceFromPoseStorage();

        follower = Constants.createFollower(hardwareMap);

        if (PoseStorage.currentPose != null) {
            follower.setStartingPose(PoseStorage.currentPose);
        } else {
            follower.setStartingPose(getTeleOpResetPose());
        }

        outtake = new Outtake();
        intake = new Intake();
        turret = new Turret();
        robotHardware = new RobotHardware(hardwareMap);

        outtake.init(hardwareMap);
        intake.init(hardwareMap);
        turret.init(hardwareMap);

        shooterEnabled = false;
        autoAimMode = START_IN_AUTO_AIM;

        updateTurretTargetFromTeleOpGoal();

        robotHardware.reset_all();
        intake.intakeStop();
        outtake.stopOuttake();

        resetVelocityEstimate(follower.getPose());

        if (!autoAimMode) {
            turret.setForward();
        }

        telemetry.addLine("Competition TeleOp Initialised");
        telemetry.addData("Alliance", allianceLabel);
        telemetry.addData("Goal X", "%.2f", TELEOP_GOAL_X);
        telemetry.addData("Goal Y", "%.2f", TELEOP_GOAL_Y);
        telemetry.addData("Reset X", "%.2f", TELEOP_RESET_X);
        telemetry.addData("Reset Y", "%.2f", TELEOP_RESET_Y);
        telemetry.addData("Reset Heading", "%.1f", TELEOP_RESET_HEADING_DEG);
        telemetry.addData("Bulk Caching", "MANUAL");
        telemetry.update();
    }

    @Override
    public void start() {
        clearBulkCache();

        follower.startTeleopDrive();

        shooterEnabled = false;
        autoAimMode = START_IN_AUTO_AIM;

        applyAllianceFromPoseStorage();
        updateTurretTargetFromTeleOpGoal();

        robotHardware.reset_all();
        intake.intakeStop();
        outtake.stopOuttake();

        resetVelocityEstimate(follower.getPose());

        if (!autoAimMode) {
            turret.setForward();
        }
    }

    @Override
    public void loop() {
        clearBulkCache();

        handlePoseReset();
        handleTurretModeControls();
        handleGamepad2TurretOffsetControls();
        handleShooterControls();

        updateTurretTargetFromTeleOpGoal();

        driveRobot();

        follower.update();

        Pose currentPose = follower.getPose();
        PoseStorage.setPose(currentPose);

        updateEstimatedFieldVelocity(currentPose);

        intake.update();

        double distCM = distanceToTeleOpGoalCM(currentPose);

        updateTurretAim(currentPose);

        if (shooterEnabled) {
            if (USE_STATIC_TARGET_TPS) {
                outtake.setTargetTPS(STATIC_TARGET_TPS);
            } else {
                outtake.linearRegression(distCM);
            }

            robotHardware.linearHoodRegression(distCM);
        } else {
            outtake.stopOuttake();
        }

        outtake.updatePIDF();

        handleIntakeAndFeedControls();

        updateTelemetry(distCM);
    }

    @Override
    public void stop() {
        clearBulkCache();

        PoseStorage.setPose(follower.getPose());

        shooterEnabled = false;
        autoAimMode = false;

        outtake.stopOuttake();
        outtake.updatePIDF();

        intake.intakeStop();
        robotHardware.reset_all();

        turret.setForward();
    }

    private void configureBulkCaching() {
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    private void clearBulkCache() {
        if (allHubs == null) {
            return;
        }

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    private void applyAllianceFromPoseStorage() {
        if (PoseStorage.isRed()) {
            allianceLabel = "RED";

            TELEOP_GOAL_X = RED_GOAL_X;
            TELEOP_GOAL_Y = RED_GOAL_Y;

            TELEOP_RESET_X = RED_RESET_X;
            TELEOP_RESET_Y = RED_RESET_Y;
            TELEOP_RESET_HEADING_DEG = RED_RESET_HEADING_DEG;

            return;
        }

        if (PoseStorage.isBlue()) {
            allianceLabel = "BLUE";

            TELEOP_GOAL_X = BLUE_GOAL_X;
            TELEOP_GOAL_Y = BLUE_GOAL_Y;

            TELEOP_RESET_X = BLUE_RESET_X;
            TELEOP_RESET_Y = BLUE_RESET_Y;
            TELEOP_RESET_HEADING_DEG = BLUE_RESET_HEADING_DEG;

            return;
        }

        allianceLabel = "UNKNOWN - DEFAULT BLUE";

        TELEOP_GOAL_X = BLUE_GOAL_X;
        TELEOP_GOAL_Y = BLUE_GOAL_Y;

        TELEOP_RESET_X = BLUE_RESET_X;
        TELEOP_RESET_Y = BLUE_RESET_Y;
        TELEOP_RESET_HEADING_DEG = BLUE_RESET_HEADING_DEG;
    }

    private void driveRobot() {
        double forward = -gamepad1.left_stick_y * DRIVE_SPEED;
        double strafe = -gamepad1.left_stick_x * DRIVE_SPEED;
        double turn = -gamepad1.right_stick_x * DRIVE_SPEED;

        if (isCollectIntakeActive()) {
            turn *= INTAKE_TURN_MULTIPLIER;
        }

        if (isFeedRequested()) {
            forward *= FEED_DRIVE_MULTIPLIER;
            strafe *= FEED_DRIVE_MULTIPLIER;
            turn *= FEED_DRIVE_MULTIPLIER;
        }

        follower.setTeleOpDrive(forward, strafe, turn);
    }

    private void handleTurretModeControls() {
        boolean currentLeftBumper = gamepad1.left_bumper;
        boolean currentDpadUp = gamepad1.dpad_up;

        if (currentLeftBumper && !previousLeftBumper) {
            autoAimMode = !autoAimMode;

            if (!autoAimMode) {
                turret.setForward();
            }
        }

        if (currentDpadUp && !previousG1DpadUp) {
            autoAimMode = true;
            updateTurretTargetFromTeleOpGoal();
        }

        previousLeftBumper = currentLeftBumper;
        previousG1DpadUp = currentDpadUp;
    }

    private void updateTurretAim(Pose currentPose) {
        if (autoAimMode) {
            turret.aimTurret(
                    currentPose,
                    estimatedFieldVelocityX,
                    estimatedFieldVelocityY
            );
        } else {
            turret.setForward();
        }
    }

    private void handleGamepad2TurretOffsetControls() {
        boolean currentG2DpadLeft = gamepad2.dpad_left;
        boolean currentG2DpadRight = gamepad2.dpad_right;
        boolean currentG2Square = gamepad2.x;

        if (currentG2DpadLeft && !previousG2DpadLeft) {
            Turret.AIM_OFFSET -= AIM_OFFSET_STEP;
            Turret.AIM_OFFSET = clamp(Turret.AIM_OFFSET, AIM_OFFSET_MIN, AIM_OFFSET_MAX);
        }

        if (currentG2DpadRight && !previousG2DpadRight) {
            Turret.AIM_OFFSET += AIM_OFFSET_STEP;
            Turret.AIM_OFFSET = clamp(Turret.AIM_OFFSET, AIM_OFFSET_MIN, AIM_OFFSET_MAX);
        }

        if (currentG2Square && !previousG2Square) {
            Turret.AIM_OFFSET = 0.0;
        }

        previousG2DpadLeft = currentG2DpadLeft;
        previousG2DpadRight = currentG2DpadRight;
        previousG2Square = currentG2Square;
    }

    private void handleShooterControls() {
        boolean currentSquare = gamepad1.x;
        boolean currentTriangle = gamepad1.y;
        boolean currentG2Triangle = gamepad2.y;

        if (currentSquare && !previousSquare) {
            shooterEnabled = true;
        }

        boolean gamepad1TrianglePressed = currentTriangle && !previousTriangle;
        boolean gamepad2TrianglePressed = currentG2Triangle && !previousG2Triangle;

        if (gamepad1TrianglePressed || gamepad2TrianglePressed) {
            resetShooterIntakeHoodAndTurret();
        }

        previousSquare = currentSquare;
        previousTriangle = currentTriangle;
        previousG2Triangle = currentG2Triangle;
    }

    private void resetShooterIntakeHoodAndTurret() {
        shooterEnabled = false;
        autoAimMode = false;

        outtake.stopOuttake();
        intake.intakeStop();

        robotHardware.reset_all();

        turret.setForward();
    }

    private void handlePoseReset() {
        boolean currentDpadRight = gamepad1.dpad_right;

        if (currentDpadRight && !previousG1DpadRight) {
            Pose resetPose = getTeleOpResetPose();

            follower.setPose(resetPose);
            PoseStorage.setPose(resetPose);

            resetVelocityEstimate(resetPose);

            autoAimMode = true;
            updateTurretTargetFromTeleOpGoal();
        }

        previousG1DpadRight = currentDpadRight;
    }

    private void handleIntakeAndFeedControls() {
        if (isFeedRequested() && shooterEnabled) {
            robotHardware.release();
            intake.intake(SHOOT_INTAKE_LEFT_POWER, SHOOT_INTAKE_RIGHT_POWER);
            return;
        }

        if (isCollectIntakeActive()) {
            robotHardware.block();
            intake.intake(COLLECT_INTAKE_LEFT_POWER, COLLECT_INTAKE_RIGHT_POWER);
            return;
        }

        robotHardware.block();
        intake.intakeStop();
    }

    private boolean isCollectIntakeActive() {
        return gamepad1.right_bumper;
    }

    private boolean isFeedRequested() {
        return gamepad1.right_trigger > SHOOT_TRIGGER_THRESHOLD;
    }

    private void updateEstimatedFieldVelocity(Pose currentPose) {
        if (!USE_POSE_DELTA_VELOCITY || currentPose == null) {
            estimatedFieldVelocityX = 0.0;
            estimatedFieldVelocityY = 0.0;
            estimatedFieldSpeed = 0.0;
            return;
        }

        double currentTime = getRuntime();

        if (previousVelocityPose == null) {
            resetVelocityEstimate(currentPose);
            return;
        }

        double dt = currentTime - previousVelocityTime;

        if (dt <= 0.001) {
            estimatedFieldVelocityX = 0.0;
            estimatedFieldVelocityY = 0.0;
            estimatedFieldSpeed = 0.0;
            return;
        }

        double dx = currentPose.getX() - previousVelocityPose.getX();
        double dy = currentPose.getY() - previousVelocityPose.getY();

        estimatedFieldVelocityX = dx / dt;
        estimatedFieldVelocityY = dy / dt;
        estimatedFieldSpeed = Math.hypot(
                estimatedFieldVelocityX,
                estimatedFieldVelocityY
        );

        previousVelocityPose = currentPose;
        previousVelocityTime = currentTime;
    }

    private void resetVelocityEstimate(Pose pose) {
        previousVelocityPose = pose;
        previousVelocityTime = getRuntime();

        estimatedFieldVelocityX = 0.0;
        estimatedFieldVelocityY = 0.0;
        estimatedFieldSpeed = 0.0;
    }

    private Pose getTeleOpGoalPose() {
        return new Pose(
                TELEOP_GOAL_X,
                TELEOP_GOAL_Y,
                0
        );
    }

    private Pose getTeleOpResetPose() {
        return new Pose(
                TELEOP_RESET_X,
                TELEOP_RESET_Y,
                Math.toRadians(TELEOP_RESET_HEADING_DEG)
        );
    }

    private void updateTurretTargetFromTeleOpGoal() {
        turret.setTargetPose(getTeleOpGoalPose());
    }

    private double distanceToTeleOpGoalCM(Pose robotPose) {
        if (robotPose == null) {
            return 0.0;
        }

        Pose goalPose = getTeleOpGoalPose();

        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();

        return Math.hypot(dx, dy) * 2.54;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private String format(String pattern, double value) {
        return String.format(Locale.US, pattern, value);
    }

    private void updateTelemetry(double distCM) {
        boolean atSpeed = outtake.isAtSpeed(0.95);

        String shooterState = shooterEnabled ? "ON" : "OFF";
        String readyState = atSpeed ? "READY" : "SPIN UP";
        String turretState = autoAimMode ? "AUTO" : "FORWARD";
        String feedState = isFeedRequested() ? "FEED" : "IDLE";

        telemetry.addLine(
                allianceLabel +
                        " | SHOOTER:" + shooterState +
                        " | " + readyState +
                        " | " + turretState +
                        " | " + feedState
        );

        telemetry.addLine(
                "DIST:" + format("%.0f", distCM) +
                        "cm | TPS:" + format("%.0f", Outtake.currentTPS) +
                        "/" + format("%.0f", Outtake.target)
        );

        telemetry.addLine(
                "OFFSET:" + format("%.3f", Turret.AIM_OFFSET) +
                        " | STEP:" + format("%.3f", AIM_OFFSET_STEP)
        );

        telemetry.update();
    }
}