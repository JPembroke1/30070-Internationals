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

@TeleOp(name = "Blue Internationals TeleOp", group = "TeleOp")
public class BlueInternationalsoftheTeleops extends OpMode {

    // ─────────────────────────────────────────────────────────────────────────
    // Subsystems
    // ─────────────────────────────────────────────────────────────────────────

    private Follower follower;
    private Intake intake;
    private Outtake outtake;
    private Turret turret;
    private RobotHardware robotHardware;

    // ─────────────────────────────────────────────────────────────────────────
    // Blue goal targeting
    //
    // Field coordinate notes:
    // - Pedro field coordinates are in inches.
    // - This TeleOp is BLUE only.
    // - Blue inset goal estimate: X = 6, Y = 138.
    //
    // Controls:
    // - gamepad1.dpad_up = force turret auto aim at blue goal.
    // - gamepad1.dpad_right = reset live robot pose near blue goal.
    // ─────────────────────────────────────────────────────────────────────────

    public static double BLUE_GOAL_X = 6;
    public static double BLUE_GOAL_Y = 138;

    // I am guessing here for the "in front of blue goal" reset pose.
    // Adjust this after checking the real field position.
    public static double BLUE_GOAL_RESET_X = 24;
    public static double BLUE_GOAL_RESET_Y = 138;
    public static double BLUE_GOAL_RESET_HEADING_DEG = 135;

    private boolean previousG1DpadUp = false;
    private boolean previousG1DpadRight = false;

    // ─────────────────────────────────────────────────────────────────────────
    // Turret aiming mode
    //
    // Controls:
    // - gamepad1.left_bumper toggles between AUTO AIM and FORWARD.
    // - gamepad1.dpad_up forces AUTO AIM.
    // - gamepad1.b forces FORWARD.
    //
    // Forward mode:
    // - Uses turret.setServoDirect(0.5).
    // - Your Turret class already supports this.
    // ─────────────────────────────────────────────────────────────────────────

    public static double FORWARD_TURRET_SERVO = 0.5;

    private boolean autoAimMode = true;
    private boolean previousLeftBumper = false;

    // ─────────────────────────────────────────────────────────────────────────
    // Drive tuning
    //
    // Controls:
    // - gamepad1.left_stick_y = forward/back.
    // - gamepad1.left_stick_x = strafe.
    // - gamepad1.right_stick_x = turn.
    // - gamepad1.right_bumper = intake collect only.
    //
    // Tuning notes:
    // - While collecting, blockers stay closed.
    // - DRIVE_SPEED controls normal driver speed.
    // - INTAKE_TURN_MULTIPLIER reduces turn only while collecting.
    // ─────────────────────────────────────────────────────────────────────────

    public static double DRIVE_SPEED = 1.0;
    public static double INTAKE_TURN_MULTIPLIER = 0.45;

    // ─────────────────────────────────────────────────────────────────────────
    // Shooter target tuning
    //
    // USE_STATIC_TARGET_TPS:
    // - true = use STATIC_TARGET_TPS.
    // - false = use distance-based linear regression.
    //
    // Controls:
    // - gamepad1.x / square = turn shooter on.
    // - gamepad1.y / triangle = everything off and hood reset.
    // - gamepad1.right_trigger = feed balls only.
    //
    // Tuning notes:
    // - Use static mode when tuning kV, kS and kP in Outtake.java.
    // - Use regression mode for normal match shooting.
    // - Hood regression runs while shooter is enabled.
    // ─────────────────────────────────────────────────────────────────────────

    public static boolean USE_STATIC_TARGET_TPS = false;
    public static double STATIC_TARGET_TPS = 1600;

    private boolean shooterEnabled = false;

    private boolean previousSquare = false;
    private boolean previousTriangle = false;

    // ─────────────────────────────────────────────────────────────────────────
    // Intake / feed tuning
    //
    // gamepad1.right_bumper:
    // - collect intake only.
    // - blockers remain closed.
    //
    // gamepad1.right_trigger:
    // - feed balls into shooter.
    // - only feeds if shooterEnabled is true.
    //
    // Tuning notes:
    // - SHOOT_INTAKE powers should feed balls smoothly into the shooter.
    // - COLLECT_INTAKE powers can be stronger for collection.
    // ─────────────────────────────────────────────────────────────────────────

    public static double SHOOT_TRIGGER_THRESHOLD = 0.2;

    public static double SHOOT_INTAKE_LEFT_POWER = 0.7;
    public static double SHOOT_INTAKE_RIGHT_POWER = 0.9;

    public static double COLLECT_INTAKE_LEFT_POWER = 1.0;
    public static double COLLECT_INTAKE_RIGHT_POWER = 1.0;

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

        shooterEnabled = false;
        autoAimMode = true;

        robotHardware.reset_all();
        intake.intakeStop();
        outtake.stopOuttake();

        telemetry.addLine("Blue TeleOp initialised");
        telemetry.addData("Goal X", "%.2f", getGoalPose().getX());
        telemetry.addData("Goal Y", "%.2f", getGoalPose().getY());
        telemetry.addData("Turret Mode", autoAimMode ? "AUTO AIM" : "FORWARD 0.5");
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
        autoAimMode = true;

        robotHardware.reset_all();
        intake.intakeStop();
        outtake.stopOuttake();

        turret.setServoDirect(FORWARD_TURRET_SERVO);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Main loop
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void loop() {
        handlePoseReset();
        handleTurretModeControls();
        handleShooterControls();

        updateGoalTarget();

        driveRobot();

        follower.update();

        Pose currentPose = follower.getPose();
        PoseStorage.currentPose = currentPose;

        intake.update();

        double distCM = distanceToGoalCM();

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

        updateTelemetry(currentPose, distCM);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Stop
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void stop() {
        PoseStorage.currentPose = follower.getPose();

        shooterEnabled = false;
        autoAimMode = false;

        outtake.stopOuttake();
        outtake.updatePIDF();

        intake.intakeStop();
        robotHardware.reset_all();

        turret.setServoDirect(FORWARD_TURRET_SERVO);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Drive controls
    //
    // Tuning notes:
    // - gamepad1.right_bumper is intake collect only.
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
    // Turret mode controls
    //
    // gamepad1.left_bumper:
    // - toggles between AUTO AIM and FORWARD.
    //
    // gamepad1.dpad_up:
    // - forces AUTO AIM.
    //
    // gamepad1.b:
    // - forces FORWARD 0.5.
    // ─────────────────────────────────────────────────────────────────────────

    private void handleTurretModeControls() {
        boolean currentLeftBumper = gamepad1.left_bumper;
        boolean currentDpadUp = gamepad1.dpad_up;

        if (currentLeftBumper && !previousLeftBumper) {
            autoAimMode = !autoAimMode;
        }

        if (currentDpadUp && !previousG1DpadUp) {
            autoAimMode = true;
            updateGoalTarget();
            turret.aimTurret(follower.getPose());
        }

        if (gamepad1.b) {
            autoAimMode = false;
            turret.setServoDirect(FORWARD_TURRET_SERVO);
        }

        previousLeftBumper = currentLeftBumper;
        previousG1DpadUp = currentDpadUp;
    }

    private void updateTurretAim(Pose currentPose) {
        if (autoAimMode) {
            turret.aimTurret(currentPose);
        } else {
            turret.setServoDirect(FORWARD_TURRET_SERVO);
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Shooter controls
    //
    // gamepad1.x / square:
    // - turns shooter on.
    // - shooter stays on until triangle/off.
    //
    // gamepad1.y / triangle:
    // - turns everything off.
    // - stops shooter.
    // - stops intake.
    // - closes blocker.
    // - resets hood to RobotSettings.close through reset_all().
    // - sets turret forward.
    // ─────────────────────────────────────────────────────────────────────────

    private void handleShooterControls() {
        boolean currentSquare = gamepad1.x;
        boolean currentTriangle = gamepad1.y;

        if (currentSquare && !previousSquare) {
            shooterEnabled = true;
        }

        if (currentTriangle && !previousTriangle) {
            shooterEnabled = false;
            autoAimMode = false;

            outtake.stopOuttake();
            intake.intakeStop();

            robotHardware.reset_all();
            turret.setServoDirect(FORWARD_TURRET_SERVO);
        }

        previousSquare = currentSquare;
        previousTriangle = currentTriangle;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Live pose reset
    //
    // gamepad1.dpad_right:
    // - resets the robot live pose to a blue-goal shooting pose.
    //
    // Important:
    // - If follower.setPose(...) fails, your Pedro version uses a different
    //   live pose reset method.
    // ─────────────────────────────────────────────────────────────────────────

    private void handlePoseReset() {
        boolean currentDpadRight = gamepad1.dpad_right;

        if (currentDpadRight && !previousG1DpadRight) {
            Pose blueGoalResetPose = new Pose(
                    BLUE_GOAL_RESET_X,
                    BLUE_GOAL_RESET_Y,
                    Math.toRadians(BLUE_GOAL_RESET_HEADING_DEG)
            );

            follower.setPose(blueGoalResetPose);
            PoseStorage.currentPose = blueGoalResetPose;

            autoAimMode = true;
            updateGoalTarget();
            turret.aimTurret(blueGoalResetPose);
        }

        previousG1DpadRight = currentDpadRight;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Intake and feed controls
    //
    // Priority order:
    // 1. gamepad1.right_trigger = feed into shooter only if shooter is on.
    // 2. gamepad1.right_bumper = collect intake with blockers closed.
    // 3. Default = intake stopped and blocker closed.
    //
    // Important:
    // - right_trigger does not start shooter.
    // - square/X starts shooter.
    // - triangle/Y stops everything.
    // ─────────────────────────────────────────────────────────────────────────

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

    // ─────────────────────────────────────────────────────────────────────────
    // Goal helpers
    //
    // Distance note:
    // - Pedro coordinates are inches.
    // - Shooter and hood regression use centimetres.
    // - Therefore distance is multiplied by 2.54.
    // ─────────────────────────────────────────────────────────────────────────

    private Pose getGoalPose() {
        return new Pose(
                BLUE_GOAL_X,
                BLUE_GOAL_Y,
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
        telemetry.addLine("Blue Internationals TeleOp - Gamepad 1 Only");

        telemetry.addLine("Controls");
        telemetry.addData("G1 Square / X", "Shooter ON");
        telemetry.addData("G1 Triangle / Y", "Everything OFF + hood reset");
        telemetry.addData("G1 Right Trigger", "Feed only");
        telemetry.addData("G1 Right Bumper", "Intake only, blockers closed");
        telemetry.addData("G1 Left Bumper", "Toggle AUTO AIM / FORWARD");
        telemetry.addData("G1 Dpad Up", "Force AUTO AIM");
        telemetry.addData("G1 Dpad Right", "Reset live pose");
        telemetry.addData("G1 B", "Force FORWARD 0.5");

        telemetry.addLine("State");
        telemetry.addData("Shooter Enabled", shooterEnabled);
        telemetry.addData("Feed Requested", isFeedRequested());
        telemetry.addData("Turret Mode", autoAimMode ? "AUTO AIM" : "FORWARD 0.5");
        telemetry.addData("Collect Intake Active", isCollectIntakeActive());
        telemetry.addData("Intake Turn Multiplier", "%.2f", INTAKE_TURN_MULTIPLIER);

        telemetry.addLine("Goal");
        telemetry.addData("Goal X", "%.2f", getGoalPose().getX());
        telemetry.addData("Goal Y", "%.2f", getGoalPose().getY());
        telemetry.addData("Distance CM", "%.1f", distCM);

        telemetry.addLine("Blue Goal Reset Pose");
        telemetry.addData("Reset X", "%.2f", BLUE_GOAL_RESET_X);
        telemetry.addData("Reset Y", "%.2f", BLUE_GOAL_RESET_Y);
        telemetry.addData("Reset Heading", "%.1f", BLUE_GOAL_RESET_HEADING_DEG);

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

        telemetry.addLine("Hood");
        telemetry.addData("Last Raw Target", "%.3f", RobotHardware.lastRawHoodTarget);
        telemetry.addData("Last Clipped Target", "%.3f", RobotHardware.lastClippedHoodTarget);
        telemetry.addData("Last Commanded", "%.3f", RobotHardware.lastCommandedHood);
        telemetry.addData("Filtered Distance", "%.1f", RobotHardware.filteredDistanceCM);
        telemetry.addData("Hood Initialised", RobotHardware.hoodInitialised);

        telemetry.addLine("Pose");
        telemetry.addData("X", "%.2f", currentPose.getX());
        telemetry.addData("Y", "%.2f", currentPose.getY());
        telemetry.addData("Heading", "%.1f", Math.toDegrees(currentPose.getHeading()));

        telemetry.addLine("Turret");
        telemetry.addData("Forward Servo", "%.3f", FORWARD_TURRET_SERVO);
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