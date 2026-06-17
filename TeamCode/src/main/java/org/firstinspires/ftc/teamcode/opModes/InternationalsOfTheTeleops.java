package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opModes.subClasses.Intake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Outtake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.RobotHardware;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Internationals Of The Teleops", group = "TeleOp")
public class InternationalsOfTheTeleops extends OpMode {

    // ─────────────────────────────────────────────────────────────────────────
    // Subsystems
    // ─────────────────────────────────────────────────────────────────────────

    private Follower follower;
    private Intake intake;
    private Outtake outtake;
    private Turret turret;
    private RobotHardware robotHardware;

    // ─────────────────────────────────────────────────────────────────────────
    // Alliance / goal targeting
    //
    // Field coordinate notes:
    // - Pedro field coordinates are in inches.
    // - Blue inset goal estimate: X = 6, Y = 138.
    // - Red inset goal estimate: X = 138, Y = 138.
    //
    // Tuning notes:
    // - gamepad2.dpad_left/right adjusts GOAL_OFFSET_X live.
    // - gamepad2.dpad_up/down adjusts GOAL_OFFSET_Y live.
    // - gamepad2.left_stick_button resets both offsets to 0.
    // ─────────────────────────────────────────────────────────────────────────

    public enum AllianceColour {
        BLUE,
        RED
    }

    public static AllianceColour allianceColour = AllianceColour.BLUE;

    public static double BLUE_GOAL_X = 6;
    public static double BLUE_GOAL_Y = 138;

    public static double RED_GOAL_X = 138;
    public static double RED_GOAL_Y = 138;

    public static double GOAL_OFFSET_X = 0;
    public static double GOAL_OFFSET_Y = 0;

    public static double GOAL_OFFSET_STEP = 0.5;

    private boolean previousG2DpadLeft = false;
    private boolean previousG2DpadRight = false;
    private boolean previousG2DpadUp = false;
    private boolean previousG2DpadDown = false;
    private boolean previousG2LeftStickButton = false;

    // ─────────────────────────────────────────────────────────────────────────
    // Drive tuning
    //
    // Control notes:
    // - gamepad1.right_bumper = collect intake.
    // - Slow mode has been removed.
    //
    // Tuning notes:
    // - DRIVE_SPEED controls normal driver speed.
    // - INTAKE_TURN_MULTIPLIER reduces turn only while intake collect is active.
    // ─────────────────────────────────────────────────────────────────────────

    public static double DRIVE_SPEED = 1.0;
    public static double INTAKE_TURN_MULTIPLIER = 0.45;

    // ─────────────────────────────────────────────────────────────────────────
    // Shooter target tuning
    //
    // USE_STATIC_TARGET_TPS:
    // - true = use STATIC_TARGET_TPS for shooter tuning.
    // - false = use distance-based linear regression.
    //
    // Tuning notes:
    // - Use static mode when tuning kV, kS and kP in Outtake.java.
    // - Use regression mode for normal match shooting.
    // - Hood regression still runs from distance in both modes.
    // ─────────────────────────────────────────────────────────────────────────

    public static boolean USE_STATIC_TARGET_TPS = true;
    public static double STATIC_TARGET_TPS = 1500;

    // ─────────────────────────────────────────────────────────────────────────
    // Intake / feed tuning
    //
    // gamepad1.right_bumper:
    // - collection intake.
    //
    // gamepad2.right_trigger:
    // - controlled shoot/feed into shooter.
    //
    // Tuning notes:
    // - SHOOT_INTAKE powers should feed balls smoothly into the shooter.
    // - COLLECT_INTAKE powers can be more aggressive for collection.
    // ─────────────────────────────────────────────────────────────────────────

    public static double SHOOT_INTAKE_LEFT_POWER = 0.7;
    public static double SHOOT_INTAKE_RIGHT_POWER = 0.9;

    public static double COLLECT_INTAKE_LEFT_POWER = 1.0;
    public static double COLLECT_INTAKE_RIGHT_POWER = 1.0;

    // ─────────────────────────────────────────────────────────────────────────
    // TeleOp state
    //
    // gamepad2.right_bumper toggles shooter on/off.
    // gamepad2.left_bumper toggles auto-aim on/off.
    // ─────────────────────────────────────────────────────────────────────────

    private boolean shooterEnabled = false;
    private boolean autoAimEnabled = true;

    private boolean previousRightBumper = false;
    private boolean previousLeftBumper = false;

    // ─────────────────────────────────────────────────────────────────────────
    // Init
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        if (PoseStorage.currentPose != null) {
            follower.setStartingPose(PoseStorage.currentPose);
        } else {
            follower.setStartingPose(new Pose(60, 80, Math.toRadians(177)));
        }

        outtake = new Outtake();
        intake = new Intake();
        turret = new Turret();
        robotHardware = new RobotHardware(hardwareMap);

        outtake.init(hardwareMap);
        intake.init(hardwareMap);
        turret.init(hardwareMap);

        updateGoalTarget();

        robotHardware.block();
        intake.intakeStop();
        outtake.stopOuttake();

        telemetry.addLine("TeleOp initialised");
        telemetry.addData("Alliance", allianceColour);
        telemetry.addData("Goal X", "%.2f", getGoalPose().getX());
        telemetry.addData("Goal Y", "%.2f", getGoalPose().getY());
        telemetry.addData("Static TPS Mode", USE_STATIC_TARGET_TPS);
        telemetry.addData("Static Target TPS", "%.0f", STATIC_TARGET_TPS);
        telemetry.update();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Start
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void start() {
        follower.startTeleopDrive();

        updateGoalTarget();

        shooterEnabled = false;
        autoAimEnabled = true;

        robotHardware.block();
        intake.intakeStop();
        outtake.stopOuttake();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Main loop
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void loop() {
        handleAllianceSelection();
        handleGoalOffsetTuning();
        handleToggles();

        updateGoalTarget();

        driveRobot();

        follower.update();

        Pose currentPose = follower.getPose();
        PoseStorage.currentPose = currentPose;

        intake.update();

        double distCM = distanceToGoalCM();

        if (autoAimEnabled) {
            turret.aimTurret(currentPose);
        }

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

        updateTelemetry(currentPose, distCM);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Stop
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void stop() {
        PoseStorage.currentPose = follower.getPose();

        outtake.stopOuttake();
        outtake.updatePIDF();

        intake.intakeStop();
        robotHardware.block();
        robotHardware.reset_all();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Drive controls
    //
    // Tuning notes:
    // - Slow mode has been removed.
    // - gamepad1.right_bumper is intake collect.
    // - While intake collect is active, only turn is reduced.
    // - Forward and strafe remain full speed.
    // ─────────────────────────────────────────────────────────────────────────

    private void driveRobot() {
        double forward = -gamepad1.left_stick_y * DRIVE_SPEED;
        double strafe = -gamepad1.left_stick_x * DRIVE_SPEED;
        double turn = -gamepad1.right_stick_x * DRIVE_SPEED;

        if (isCollectIntakeActive()) {
            turn *= INTAKE_TURN_MULTIPLIER;
        }

        follower.setTeleOpDrive(forward, strafe, turn);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Toggles and manual overrides
    // ─────────────────────────────────────────────────────────────────────────

    private void handleToggles() {
        boolean currentRightBumper = gamepad2.right_bumper;
        boolean currentLeftBumper = gamepad2.left_bumper;

        if (currentRightBumper && !previousRightBumper) {
            shooterEnabled = !shooterEnabled;
        }

        if (currentLeftBumper && !previousLeftBumper) {
            autoAimEnabled = !autoAimEnabled;
        }

        previousRightBumper = currentRightBumper;
        previousLeftBumper = currentLeftBumper;

        if (gamepad2.b) {
            turret.centre();
        }

        if (gamepad2.a) {
            shooterEnabled = false;
            outtake.stopOuttake();
            robotHardware.block();
            intake.intakeStop();
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Alliance selection
    // ─────────────────────────────────────────────────────────────────────────

    private void handleAllianceSelection() {
        if (gamepad1.dpad_left) {
            allianceColour = AllianceColour.BLUE;
        }

        if (gamepad1.dpad_right) {
            allianceColour = AllianceColour.RED;
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Live goal offset tuning
    // ─────────────────────────────────────────────────────────────────────────

    private void handleGoalOffsetTuning() {
        boolean currentDpadLeft = gamepad2.dpad_left;
        boolean currentDpadRight = gamepad2.dpad_right;
        boolean currentDpadUp = gamepad2.dpad_up;
        boolean currentDpadDown = gamepad2.dpad_down;
        boolean currentLeftStickButton = gamepad2.left_stick_button;

        if (currentDpadLeft && !previousG2DpadLeft) {
            GOAL_OFFSET_X -= GOAL_OFFSET_STEP;
        }

        if (currentDpadRight && !previousG2DpadRight) {
            GOAL_OFFSET_X += GOAL_OFFSET_STEP;
        }

        if (currentDpadDown && !previousG2DpadDown) {
            GOAL_OFFSET_Y -= GOAL_OFFSET_STEP;
        }

        if (currentDpadUp && !previousG2DpadUp) {
            GOAL_OFFSET_Y += GOAL_OFFSET_STEP;
        }

        if (currentLeftStickButton && !previousG2LeftStickButton) {
            GOAL_OFFSET_X = 0;
            GOAL_OFFSET_Y = 0;
        }

        previousG2DpadLeft = currentDpadLeft;
        previousG2DpadRight = currentDpadRight;
        previousG2DpadUp = currentDpadUp;
        previousG2DpadDown = currentDpadDown;
        previousG2LeftStickButton = currentLeftStickButton;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Intake and feed controls
    //
    // Priority order:
    // 1. gamepad2.right_trigger = shoot/feed mode.
    // 2. gamepad1.right_bumper = collect mode.
    // 3. gamepad2.x = stop and block.
    // 4. gamepad2.y = release blocker only.
    // 5. Default = intake stopped and blocker closed.
    // ─────────────────────────────────────────────────────────────────────────

    private void handleIntakeAndFeedControls() {
        if (gamepad2.right_trigger > 0.2) {
            robotHardware.release();
            intake.intake(SHOOT_INTAKE_LEFT_POWER, SHOOT_INTAKE_RIGHT_POWER);
            return;
        }

        if (isCollectIntakeActive()) {
            robotHardware.release();
            intake.intake(COLLECT_INTAKE_LEFT_POWER, COLLECT_INTAKE_RIGHT_POWER);
            return;
        }

        if (gamepad2.x) {
            robotHardware.block();
            intake.intakeStop();
            return;
        }

        if (gamepad2.y) {
            robotHardware.release();
            return;
        }

        robotHardware.block();
        intake.intakeStop();
    }

    private boolean isCollectIntakeActive() {
        return gamepad1.right_bumper;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Goal helpers
    //
    // Distance note:
    // - Pedro coordinates are inches.
    // - Shooter and hood regression use centimetres.
    // - Therefore distance is multiplied by 2.54.
    // ─────────────────────────────────────────────────────────────────────────

    private Pose getGoalPose() {
        if (allianceColour == AllianceColour.RED) {
            return new Pose(
                    RED_GOAL_X + GOAL_OFFSET_X,
                    RED_GOAL_Y + GOAL_OFFSET_Y,
                    0
            );
        }

        return new Pose(
                BLUE_GOAL_X + GOAL_OFFSET_X,
                BLUE_GOAL_Y + GOAL_OFFSET_Y,
                0
        );
    }

    private void updateGoalTarget() {
        turret.setPose(getGoalPose());
    }

    private double distanceToGoalCM() {
        Pose robotPose = follower.getPose();
        Pose goalPose = getGoalPose();

        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();

        return Math.hypot(dx, dy) * 2.54;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Telemetry
    // ─────────────────────────────────────────────────────────────────────────

    private void updateTelemetry(Pose currentPose, double distCM) {
        telemetry.addData("Alliance", allianceColour);
        telemetry.addData("Shooter Enabled", shooterEnabled);
        telemetry.addData("Auto Aim Enabled", autoAimEnabled);
        telemetry.addData("Collect Intake Active", isCollectIntakeActive());
        telemetry.addData("Intake Turn Multiplier", "%.2f", INTAKE_TURN_MULTIPLIER);

        telemetry.addLine("Goal");
        telemetry.addData("Goal X", "%.2f", getGoalPose().getX());
        telemetry.addData("Goal Y", "%.2f", getGoalPose().getY());
        telemetry.addData("Goal Offset X", "%.2f", GOAL_OFFSET_X);
        telemetry.addData("Goal Offset Y", "%.2f", GOAL_OFFSET_Y);
        telemetry.addData("Offset Step", "%.2f", GOAL_OFFSET_STEP);
        telemetry.addData("Distance CM", "%.1f", distCM);

        telemetry.addLine("Shooter Target Mode");
        telemetry.addData("Static TPS Mode", USE_STATIC_TARGET_TPS);
        telemetry.addData("Static Target TPS", "%.0f", STATIC_TARGET_TPS);
        telemetry.addData("Regression Slope", "%.4f", Outtake.regressionSlope);
        telemetry.addData("Regression Intercept", "%.1f", Outtake.regressionIntercept);

        telemetry.addLine("Shooter");
        telemetry.addData("Target TPS", "%.0f", Outtake.target);
        telemetry.addData("Current TPS", "%.0f", Outtake.currentTPS);
        telemetry.addData("Left TPS", "%.0f", Outtake.leftVelocity);
        telemetry.addData("Right TPS", "%.0f", Outtake.rightVelocity);
        telemetry.addData("Error", "%.0f", Outtake.lastError);
        telemetry.addData("Output", "%.3f", Outtake.lastOutput);
        telemetry.addData("At Speed %", outtake.isAtSpeed(0.95));
        telemetry.addData("At Speed Tol", outtake.isAtSpeedTolerance());

        telemetry.addLine("Pose");
        telemetry.addData("X", "%.2f", currentPose.getX());
        telemetry.addData("Y", "%.2f", currentPose.getY());
        telemetry.addData("Heading", "%.1f", Math.toDegrees(currentPose.getHeading()));

        telemetry.addLine("Turret");
        telemetry.addData("Relative Angle", "%.1f", Turret.relativeAngleDeg);
        telemetry.addData("Desired Servo", "%.3f", Turret.desiredServo);
        telemetry.addData("Current Servo", "%.3f", Turret.currentServo);
        telemetry.addData("Servo Error", "%.4f", Turret.servoError);
        telemetry.addData("Correction", "%.4f", Turret.appliedCorrection);
        telemetry.addData("Dynamic Step", "%.4f", Turret.dynamicMaxStep);
        telemetry.addData("At Target", turret.isAtTarget());

        telemetry.addLine("Intake Sensor / LED");
        telemetry.addData("Raw Sensor", Intake.rawSensorState);
        telemetry.addData("Ball Detected", Intake.ballDetected);
        telemetry.addData("Detected Time", "%.2f", Intake.detectedTimeSeconds);
        telemetry.addData("LED Position", "%.3f", Intake.lastLedPosition);

        telemetry.update();
    }
}