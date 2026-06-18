package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opModes.subClasses.Intake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Outtake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.RobotHardware;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "Blue Internationals TeleOp", group = "TeleOp")
public class BlueTeleOp extends OpMode {

    private Follower follower;
    private Intake intake;
    private Outtake outtake;
    private Turret turret;
    private RobotHardware robotHardware;

    // ─────────────────────────────────────────────────────────────────────────
    // TeleOp goal tuneables
    //
    // These are the main target values for this TeleOp.
    // The turret target and shooter distance both use these values.
    //
    // If the turret is aiming left/right of the goal, tune TELEOP_GOAL_X.
    // If the distance/hood feels wrong, tune TELEOP_GOAL_Y or hood regression.
    // ─────────────────────────────────────────────────────────────────────────

    public static double TELEOP_GOAL_X = 6.0;
    public static double TELEOP_GOAL_Y = 138.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Pose reset tuneables
    //
    // gamepad1.dpad_right resets the live robot pose to this pose.
    //
    // These values should represent the robot being in front of the blue goal.
    // Heading is in degrees for easier tuning.
    // ─────────────────────────────────────────────────────────────────────────

    public static double TELEOP_RESET_X = 24.0;
    public static double TELEOP_RESET_Y = 138.0;
    public static double TELEOP_RESET_HEADING_DEG = 177.0;

    private boolean previousG1DpadUp = false;
    private boolean previousG1DpadRight = false;

    // ─────────────────────────────────────────────────────────────────────────
    // Turret tuneables
    //
    // FORWARD_TURRET_SERVO:
    // - Used when turret is in straight-ahead mode.
    // - 0.5 is centre based on your Turret class.
    //
    // START_IN_AUTO_AIM:
    // - true = TeleOp starts with turret tracking goal.
    // - false = TeleOp starts with turret facing forward.
    // ─────────────────────────────────────────────────────────────────────────

    public static double FORWARD_TURRET_SERVO = 0.5;
    public static boolean START_IN_AUTO_AIM = true;

    private boolean autoAimMode = true;
    private boolean previousLeftBumper = false;

    // ─────────────────────────────────────────────────────────────────────────
    // Drive tuneables
    // ─────────────────────────────────────────────────────────────────────────

    public static double DRIVE_SPEED = 1.0;
    public static double INTAKE_TURN_MULTIPLIER = 0.45;

    // ─────────────────────────────────────────────────────────────────────────
    // Shooter tuneables
    //
    // USE_STATIC_TARGET_TPS:
    // - true = use STATIC_TARGET_TPS.
    // - false = use Outtake linear regression.
    //
    // For match use, this will usually be false.
    // For kV/kS/kP tuning, this can be true.
    // ─────────────────────────────────────────────────────────────────────────

    public static boolean USE_STATIC_TARGET_TPS = false;
    public static double STATIC_TARGET_TPS = 1500.0;

    private boolean shooterEnabled = false;

    private boolean previousSquare = false;
    private boolean previousTriangle = false;

    // ─────────────────────────────────────────────────────────────────────────
    // Intake / feed tuneables
    // ─────────────────────────────────────────────────────────────────────────

    public static double SHOOT_TRIGGER_THRESHOLD = 0.2;

    public static double SHOOT_INTAKE_LEFT_POWER = 0.7;
    public static double SHOOT_INTAKE_RIGHT_POWER = 0.9;

    public static double COLLECT_INTAKE_LEFT_POWER = 1.0;
    public static double COLLECT_INTAKE_RIGHT_POWER = 1.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Telemetry tuneables
    // ─────────────────────────────────────────────────────────────────────────

    public static boolean SHOW_FULL_TELEMETRY = true;

    // ─────────────────────────────────────────────────────────────────────────
    // Init
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        if (PoseStorage.currentPose != null) {
            follower.setStartingPose(PoseStorage.currentPose);
        } else {
            follower.setStartingPose(new Pose(24, 138, Math.toRadians(177)));
        }

        outtake = new Outtake();
        intake = new Intake();
        turret = new Turret();
        robotHardware = new RobotHardware(hardwareMap);

        outtake.init(hardwareMap);
        intake.init(hardwareMap);
        turret.init(hardwareMap, follower::getAngularVelocity);

        shooterEnabled = false;
        autoAimMode = START_IN_AUTO_AIM;

        updateTurretTargetFromTeleOpGoal();

        robotHardware.reset_all();
        intake.intakeStop();
        outtake.stopOuttake();

        if (!autoAimMode) {
            turret.setServoDirect(FORWARD_TURRET_SERVO);
        }

        telemetry.addLine("Blue TeleOp initialised");
        telemetry.addData("TeleOp Goal X", "%.2f", TELEOP_GOAL_X);
        telemetry.addData("TeleOp Goal Y", "%.2f", TELEOP_GOAL_Y);
        telemetry.addData("Turret Mode", autoAimMode ? "AUTO AIM" : "FORWARD");
        telemetry.addData("Forward Servo", "%.3f", FORWARD_TURRET_SERVO);
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

        shooterEnabled = false;
        autoAimMode = START_IN_AUTO_AIM;

        updateTurretTargetFromTeleOpGoal();

        robotHardware.reset_all();
        intake.intakeStop();
        outtake.stopOuttake();

        if (!autoAimMode) {
            turret.setServoDirect(FORWARD_TURRET_SERVO);
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Main loop
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void loop() {
        handlePoseReset();
        handleTurretModeControls();
        handleShooterControls();

        updateTurretTargetFromTeleOpGoal();

        driveRobot();

        follower.update();

        Pose currentPose = follower.getPose();
        PoseStorage.currentPose = currentPose;

        intake.update();

        double distCM = distanceToTeleOpGoalCM();

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
    // gamepad1.left_stick_y = forward/back
    // gamepad1.left_stick_x = strafe
    // gamepad1.right_stick_x = turn
    //
    // gamepad1.right_bumper = intake only, blockers closed
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
    // Turret controls
    //
    // gamepad1.left_bumper:
    // - Toggle between AUTO AIM and FORWARD.
    //
    // gamepad1.dpad_up:
    // - Force AUTO AIM.
    //
    // gamepad1.b:
    // - Force FORWARD.
    // ─────────────────────────────────────────────────────────────────────────

    private void handleTurretModeControls() {
        boolean currentLeftBumper = gamepad1.left_bumper;
        boolean currentDpadUp = gamepad1.dpad_up;

        if (currentLeftBumper && !previousLeftBumper) {
            autoAimMode = !autoAimMode;

            if (!autoAimMode) {
                turret.setServoDirect(FORWARD_TURRET_SERVO);
            }
        }

        if (currentDpadUp && !previousG1DpadUp) {
            autoAimMode = true;
            updateTurretTargetFromTeleOpGoal();
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
    // - Shooter on.
    //
    // gamepad1.y / triangle:
    // - Everything off.
    // - Hood reset.
    // - Turret forward.
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
    // Pose reset
    //
    // gamepad1.dpad_right:
    // - Reset live pose near blue goal.
    //
    // If follower.setPose(...) errors, your Pedro version uses a different
    // pose reset method.
    // ─────────────────────────────────────────────────────────────────────────

    private void handlePoseReset() {
        boolean currentDpadRight = gamepad1.dpad_right;

        if (currentDpadRight && !previousG1DpadRight) {
            Pose resetPose = getTeleOpResetPose();

            follower.setPose(resetPose);
            PoseStorage.currentPose = resetPose;

            autoAimMode = true;
            updateTurretTargetFromTeleOpGoal();
            turret.aimTurret(resetPose);
        }

        previousG1DpadRight = currentDpadRight;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Intake and feed controls
    //
    // gamepad1.right_trigger:
    // - Feed only if shooter is already on.
    //
    // gamepad1.right_bumper:
    // - Intake collect only.
    // - Blockers stay closed.
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
    // TeleOp target helpers
    //
    // This is the important part:
    // - TeleOp owns the goal target.
    // - Turret target is updated from this TeleOp target.
    // - Distance calculation also uses this TeleOp target.
    // ─────────────────────────────────────────────────────────────────────────

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

    private double distanceToTeleOpGoalCM() {
        Pose robotPose = follower.getPose();
        Pose goalPose = getTeleOpGoalPose();

        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();

        return Math.hypot(dx, dy) * 2.54;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Telemetry
    // ─────────────────────────────────────────────────────────────────────────

    private void updateTelemetry(Pose currentPose, double distCM) {
        telemetry.addLine("Blue Internationals TeleOp - Configurable");

        telemetry.addLine("Controls");
        telemetry.addData("G1 Square / X", "Shooter ON");
        telemetry.addData("G1 Triangle / Y", "Everything OFF + hood reset");
        telemetry.addData("G1 Right Trigger", "Feed only");
        telemetry.addData("G1 Right Bumper", "Intake only, blockers closed");
        telemetry.addData("G1 Left Bumper", "Toggle AUTO AIM / FORWARD");
        telemetry.addData("G1 Dpad Up", "Force AUTO AIM");
        telemetry.addData("G1 Dpad Right", "Reset live pose");
        telemetry.addData("G1 B", "Force FORWARD");

        telemetry.addLine("State");
        telemetry.addData("Shooter Enabled", shooterEnabled);
        telemetry.addData("Feed Requested", isFeedRequested());
        telemetry.addData("Turret Mode", autoAimMode ? "AUTO AIM" : "FORWARD");
        telemetry.addData("Collect Intake Active", isCollectIntakeActive());

        telemetry.addLine("TeleOp Tuneables");
        telemetry.addData("TeleOp Goal X", "%.2f", TELEOP_GOAL_X);
        telemetry.addData("TeleOp Goal Y", "%.2f", TELEOP_GOAL_Y);
        telemetry.addData("Reset X", "%.2f", TELEOP_RESET_X);
        telemetry.addData("Reset Y", "%.2f", TELEOP_RESET_Y);
        telemetry.addData("Reset Heading", "%.1f", TELEOP_RESET_HEADING_DEG);
        telemetry.addData("Forward Turret Servo", "%.3f", FORWARD_TURRET_SERVO);
        telemetry.addData("Start In Auto Aim", START_IN_AUTO_AIM);
        telemetry.addData("Drive Speed", "%.2f", DRIVE_SPEED);
        telemetry.addData("Intake Turn Multiplier", "%.2f", INTAKE_TURN_MULTIPLIER);
        telemetry.addData("Shoot Trigger Threshold", "%.2f", SHOOT_TRIGGER_THRESHOLD);

        telemetry.addLine("Goal / Distance");
        telemetry.addData("Target X", "%.2f", getTeleOpGoalPose().getX());
        telemetry.addData("Target Y", "%.2f", getTeleOpGoalPose().getY());
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
        telemetry.addData("Turret Target X", "%.2f", Turret.targetPose.getX());
        telemetry.addData("Turret Target Y", "%.2f", Turret.targetPose.getY());
        telemetry.addData("Relative Angle", "%.1f", Turret.relativeAngleDeg);
        telemetry.addData("Desired Servo", "%.3f", Turret.desiredServo);
        telemetry.addData("Current Servo", "%.3f", Turret.currentServo);
        telemetry.addData("Servo Error", "%.4f", Turret.servoError);
        telemetry.addData("Correction", "%.4f", Turret.appliedCorrection);
        telemetry.addData("Dynamic Step", "%.4f", Turret.dynamicMaxStep);
        telemetry.addData("At Target", turret.isAtTarget());

        if (SHOW_FULL_TELEMETRY) {
            telemetry.addLine("Intake Sensor / LED");
            telemetry.addData("Raw Sensor", Intake.rawSensorState);
            telemetry.addData("Ball Detected", Intake.ballDetected);
            telemetry.addData("Detected Time", "%.2f", Intake.detectedTimeSeconds);
            telemetry.addData("LED Position", "%.3f", Intake.lastLedPosition);

            telemetry.addLine("Intake Powers");
            telemetry.addData("Collect Left", "%.2f", COLLECT_INTAKE_LEFT_POWER);
            telemetry.addData("Collect Right", "%.2f", COLLECT_INTAKE_RIGHT_POWER);
            telemetry.addData("Shoot Left", "%.2f", SHOOT_INTAKE_LEFT_POWER);
            telemetry.addData("Shoot Right", "%.2f", SHOOT_INTAKE_RIGHT_POWER);
        }

        telemetry.update();
    }
}