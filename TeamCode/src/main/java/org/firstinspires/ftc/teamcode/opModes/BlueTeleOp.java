package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opModes.subClasses.*;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.Locale;

@Configurable
@TeleOp(name = "Blue Internationals TeleOp", group = "TeleOp")
public class BlueTeleOp extends OpMode {

    private Follower follower;
    private Intake intake;
    private Outtake outtake;
    private Turret turret;
    private RobotHardware robotHardware;

    private List<LynxModule> allHubs;

    public static double TELEOP_GOAL_X = 6.0;
    public static double TELEOP_GOAL_Y = 138.0;

    public static double TELEOP_RESET_X = 24.0;
    public static double TELEOP_RESET_Y = 138.0;
    public static double TELEOP_RESET_HEADING_DEG = 145.0;

    private boolean previousG1DpadUp = false;
    private boolean previousG1DpadRight = false;

    public static boolean START_IN_AUTO_AIM = true;
    private boolean autoAimMode = true;
    private boolean previousLeftBumper = false;

    public static double AIM_OFFSET_STEP = 0.005;
    public static double AIM_OFFSET_MIN = -0.20;
    public static double AIM_OFFSET_MAX = 0.20;

    private boolean previousG2DpadLeft = false;
    private boolean previousG2DpadRight = false;
    private boolean previousG2Square = false;

    public static double DRIVE_SPEED = 1.0;
    public static double INTAKE_TURN_MULTIPLIER = 0.45;
    public static double FEED_DRIVE_MULTIPLIER = 0.50;

    public static boolean USE_STATIC_TARGET_TPS = false;
    public static double STATIC_TARGET_TPS = 1500.0;

    private boolean shooterEnabled = false;

    private boolean previousSquare = false;
    private boolean previousTriangle = false;
    private boolean previousG2Triangle = false;

    public static double SHOOT_TRIGGER_THRESHOLD = 0.2;

    public static double SHOOT_INTAKE_LEFT_POWER = 0.65;
    public static double SHOOT_INTAKE_RIGHT_POWER = 0.65;

    public static double COLLECT_INTAKE_LEFT_POWER = 1.0;
    public static double COLLECT_INTAKE_RIGHT_POWER = 1.0;

    private Pose previousVelocityPose = null;
    private double previousVelocityTime = 0.0;

    private double estimatedFieldVelocityX = 0.0;
    private double estimatedFieldVelocityY = 0.0;

    // ─────────────────────────────────────────
    // Bulk caching
    // ─────────────────────────────────────────

    private void configureBulkCaching() {
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    private void clearBulkCache() {
        if (allHubs == null) return;

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    @Override
    public void init() {
        configureBulkCaching();
        clearBulkCache();

        follower = Constants.createFollower(hardwareMap);

        if (PoseStorage.currentPose != null) {
            follower.setStartingPose(PoseStorage.currentPose);
        } else {
            follower.setStartingPose(new Pose(24, 130, Math.toRadians(145)));
        }

        outtake = new Outtake();
        intake = new Intake();
        turret = new Turret();
        robotHardware = new RobotHardware(hardwareMap);

        outtake.init(hardwareMap);
        intake.init(hardwareMap);
        turret.init(hardwareMap);

        autoAimMode = START_IN_AUTO_AIM;

        robotHardware.reset_all();
        intake.intakeStop();
        outtake.stopOuttake();

        resetVelocityEstimate(follower.getPose());

        if (!autoAimMode) turret.setForward();

        updateTurretTargetFromTeleOpGoal();
    }

    @Override
    public void start() {
        clearBulkCache();

        follower.startTeleopDrive();

        shooterEnabled = false;
        autoAimMode = START_IN_AUTO_AIM;

        robotHardware.reset_all();
        intake.intakeStop();
        outtake.stopOuttake();

        resetVelocityEstimate(follower.getPose());

        if (!autoAimMode) turret.setForward();

        updateTurretTargetFromTeleOpGoal();
    }

    @Override
    public void loop() {
        clearBulkCache();

        handlePoseReset();
        handleTurretModeControls();
        handleGamepad2TurretOffsetControls();
        handleShooterControls();

        driveRobot();

        follower.update();

        Pose currentPose = follower.getPose();
        PoseStorage.setPose(currentPose);

        updateEstimatedFieldVelocity(currentPose);

        double distCM = distanceToTeleOpGoalCM(currentPose);

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

        updateTurretAim(currentPose);
        handleIntakeAndFeedControls();

        updateTelemetry(distCM);
    }

    @Override
    public void stop() {
        clearBulkCache();

        PoseStorage.setPose(follower.getPose());

        outtake.stopOuttake();
        intake.intakeStop();
        robotHardware.reset_all();

        turret.setForward();
    }

    // ─────────────────────────────────────────
    // Driving
    // ─────────────────────────────────────────

    private void driveRobot() {
        double forward = -gamepad1.left_stick_y * DRIVE_SPEED;
        double strafe = -gamepad1.left_stick_x * DRIVE_SPEED;
        double turn = -gamepad1.right_stick_x * DRIVE_SPEED;

        if (gamepad1.right_bumper) {
            turn *= INTAKE_TURN_MULTIPLIER;
        }

        if (gamepad1.right_trigger > SHOOT_TRIGGER_THRESHOLD) {
            forward *= FEED_DRIVE_MULTIPLIER;
            strafe *= FEED_DRIVE_MULTIPLIER;
            turn *= FEED_DRIVE_MULTIPLIER;
        }

        follower.setTeleOpDrive(forward, strafe, turn);
    }

    // ─────────────────────────────────────────
    // Turret
    // ─────────────────────────────────────────

    private void handleTurretModeControls() {
        if (gamepad1.left_bumper && !previousLeftBumper) {
            autoAimMode = !autoAimMode;
            if (!autoAimMode) turret.setForward();
        }

        if (gamepad1.dpad_up && !previousG1DpadUp) {
            autoAimMode = true;
        }

        previousLeftBumper = gamepad1.left_bumper;
        previousG1DpadUp = gamepad1.dpad_up;
    }

    private void updateTurretAim(Pose pose) {
        if (autoAimMode) {
            turret.aimTurret(pose, estimatedFieldVelocityX, estimatedFieldVelocityY);
        }
    }

    private void handleGamepad2TurretOffsetControls() {
        if (gamepad2.dpad_left && !previousG2DpadLeft) {
            Turret.AIM_OFFSET = clamp(Turret.AIM_OFFSET - AIM_OFFSET_STEP, AIM_OFFSET_MIN, AIM_OFFSET_MAX);
        }

        if (gamepad2.dpad_right && !previousG2DpadRight) {
            Turret.AIM_OFFSET = clamp(Turret.AIM_OFFSET + AIM_OFFSET_STEP, AIM_OFFSET_MIN, AIM_OFFSET_MAX);
        }

        if (gamepad2.x && !previousG2Square) {
            Turret.AIM_OFFSET = 0.0;
        }

        previousG2DpadLeft = gamepad2.dpad_left;
        previousG2DpadRight = gamepad2.dpad_right;
        previousG2Square = gamepad2.x;
    }

    // ✅ FIXED missing method
    private Pose getTeleOpGoalPose() {
        return new Pose(TELEOP_GOAL_X, TELEOP_GOAL_Y, 0);
    }

    private void updateTurretTargetFromTeleOpGoal() {
        turret.setTargetPose(getTeleOpGoalPose());
    }

    // ─────────────────────────────────────────
    // Shooter
    // ─────────────────────────────────────────

    private void handleShooterControls() {
        if (gamepad1.x && !previousSquare) {
            shooterEnabled = true;
        }

        if ((gamepad1.y && !previousTriangle) || (gamepad2.y && !previousG2Triangle)) {
            shooterEnabled = false;
            intake.intakeStop();
            outtake.stopOuttake();
            turret.setForward();
        }

        previousSquare = gamepad1.x;
        previousTriangle = gamepad1.y;
        previousG2Triangle = gamepad2.y;
    }

    // ─────────────────────────────────────────
    // Pose reset
    // ─────────────────────────────────────────

    private void handlePoseReset() {
        if (gamepad1.dpad_right && !previousG1DpadRight) {
            Pose resetPose = new Pose(
                    TELEOP_RESET_X,
                    TELEOP_RESET_Y,
                    Math.toRadians(TELEOP_RESET_HEADING_DEG)
            );

            follower.setPose(resetPose);
            PoseStorage.setPose(resetPose);
            resetVelocityEstimate(resetPose);
            autoAimMode = true;
        }

        previousG1DpadRight = gamepad1.dpad_right;
    }

    // ─────────────────────────────────────────
    // Intake
    // ─────────────────────────────────────────

    private void handleIntakeAndFeedControls() {
        if (gamepad1.right_trigger > SHOOT_TRIGGER_THRESHOLD && shooterEnabled) {
            robotHardware.release();
            intake.intake(SHOOT_INTAKE_LEFT_POWER, SHOOT_INTAKE_RIGHT_POWER);
            return;
        }

        if (gamepad1.right_bumper) {
            robotHardware.block();
            intake.intake(COLLECT_INTAKE_LEFT_POWER, COLLECT_INTAKE_RIGHT_POWER);
            return;
        }

        robotHardware.block();
        intake.intakeStop();
    }

    // ─────────────────────────────────────────
    // Velocity
    // ─────────────────────────────────────────

    private void updateEstimatedFieldVelocity(Pose pose) {
        double now = getRuntime();

        if (previousVelocityPose == null) {
            resetVelocityEstimate(pose);
            return;
        }

        double dt = now - previousVelocityTime;
        if (dt <= 0.001) return;

        estimatedFieldVelocityX = (pose.getX() - previousVelocityPose.getX()) / dt;
        estimatedFieldVelocityY = (pose.getY() - previousVelocityPose.getY()) / dt;

        previousVelocityPose = pose;
        previousVelocityTime = now;
    }

    private void resetVelocityEstimate(Pose pose) {
        previousVelocityPose = pose;
        previousVelocityTime = getRuntime();
    }

    // ─────────────────────────────────────────
    // Distance
    // ─────────────────────────────────────────

    private double distanceToTeleOpGoalCM(Pose pose) {
        double dx = TELEOP_GOAL_X - pose.getX();
        double dy = TELEOP_GOAL_Y - pose.getY();
        return Math.hypot(dx, dy) * 2.54;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    // ─────────────────────────────────────────
    // Telemetry
    // ─────────────────────────────────────────

    private void updateTelemetry(double distCM) {
        boolean atSpeed = outtake.isAtSpeed(0.95);

        telemetry.addLine(
                "SHOOTER:" + (shooterEnabled ? "ON" : "OFF") +
                        " | " + (atSpeed ? "READY" : "SPIN UP")
        );

        telemetry.addLine(
                "DIST:" + String.format(Locale.US, "%.0f", distCM) +
                        " TPS:" + String.format(Locale.US, "%.0f", Outtake.currentTPS)
        );

        telemetry.update();
    }
}