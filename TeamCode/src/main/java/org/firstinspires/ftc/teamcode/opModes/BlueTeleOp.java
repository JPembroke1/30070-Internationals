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

    // ─────────────────────────────────────────────────────────────────────────
    // Subsystems
    // ─────────────────────────────────────────────────────────────────────────

    private Follower follower;
    private Intake intake;
    private Outtake outtake;
    private Turret turret;
    private RobotHardware robotHardware;

    // ─────────────────────────────────────────────────────────────────────────
    // TeleOp goal tuneables
    //
    // These are the single source of truth for:
    // - turret target
    // - shooter distance
    // - hood regression distance
    // ─────────────────────────────────────────────────────────────────────────

    public static double TELEOP_GOAL_X = 6.0;
    public static double TELEOP_GOAL_Y = 138.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Pose reset tuneables
    //
    // gamepad1.dpad_right resets the live robot pose to this pose.
    // Heading is in degrees for easier tuning.
    // ─────────────────────────────────────────────────────────────────────────

    public static double TELEOP_RESET_X = 24.0;
    public static double TELEOP_RESET_Y = 138.0;
    public static double TELEOP_RESET_HEADING_DEG = 145.0;

    private boolean previousG1DpadUp = false;
    private boolean previousG1DpadRight = false;

    // ─────────────────────────────────────────────────────────────────────────
    // Turret mode tuneables
    //
    // START_IN_AUTO_AIM:
    // - true = TeleOp starts with turret tracking the goal.
    // - false = TeleOp starts with turret facing forward.
    //
    // Forward servo is now tuned in Turret.java:
    // - Turret.FORWARD_SERVO
    // ─────────────────────────────────────────────────────────────────────────

    public static boolean START_IN_AUTO_AIM = true;

    private boolean autoAimMode = true;
    private boolean previousLeftBumper = false;

    // ─────────────────────────────────────────────────────────────────────────
    // Drive tuneables
    //
    // DRIVE_SPEED:
    // - normal driver speed.
    //
    // INTAKE_TURN_MULTIPLIER:
    // - reduces turn while collecting with right bumper.
    //
    // FEED_DRIVE_MULTIPLIER:
    // - reduces all movement while right trigger is held.
    // ─────────────────────────────────────────────────────────────────────────

    public static double DRIVE_SPEED = 1.0;
    public static double INTAKE_TURN_MULTIPLIER = 0.45;
    public static double FEED_DRIVE_MULTIPLIER = 0.50;

    // ─────────────────────────────────────────────────────────────────────────
    // Shooter tuneables
    //
    // USE_STATIC_TARGET_TPS:
    // - true = use STATIC_TARGET_TPS.
    // - false = use Outtake linear regression.
    // ─────────────────────────────────────────────────────────────────────────

    public static boolean USE_STATIC_TARGET_TPS = false;
    public static double STATIC_TARGET_TPS = 1500.0;

    private boolean shooterEnabled = false;

    private boolean previousSquare = false;
    private boolean previousTriangle = false;

    // ─────────────────────────────────────────────────────────────────────────
    // Intake / feed tuneables
    //
    // gamepad1.right_trigger:
    // - feed mode.
    // - feed only works if shooterEnabled is true.
    // - feed mode also slows the drive base.
    //
    // gamepad1.right_bumper:
    // - intake collect only.
    // - blockers stay closed.
    // ─────────────────────────────────────────────────────────────────────────

    public static double SHOOT_TRIGGER_THRESHOLD = 0.2;

    public static double SHOOT_INTAKE_LEFT_POWER = 0.5;
    public static double SHOOT_INTAKE_RIGHT_POWER = 0.5;

    public static double COLLECT_INTAKE_LEFT_POWER = 1.0;
    public static double COLLECT_INTAKE_RIGHT_POWER = 1.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Field velocity estimation
    //
    // Pedro coordinates are in inches.
    // This estimates field velocity using pose delta:
    //
    // velocityX = deltaX / deltaTime
    // velocityY = deltaY / deltaTime
    //
    // These values are passed into Turret.aimTurret(currentPose, vx, vy).
    // ─────────────────────────────────────────────────────────────────────────

    public static boolean USE_POSE_DELTA_VELOCITY = true;

    private Pose previousVelocityPose = null;
    private double previousVelocityTime = 0.0;

    private double estimatedFieldVelocityX = 0.0;
    private double estimatedFieldVelocityY = 0.0;
    private double estimatedFieldSpeed = 0.0;

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
            follower.setStartingPose(new Pose(24, 138, Math.toRadians(145)));
        }

        outtake = new Outtake();
        intake = new Intake();
        turret = new Turret();
        robotHardware = new RobotHardware(hardwareMap);

        outtake.init(hardwareMap);
        intake.init(hardwareMap);

        // Angular velocity comes directly from the follower.
        // Translational X/Y velocity is estimated in this TeleOp using pose delta.
        turret.init(hardwareMap, follower::getAngularVelocity);

        shooterEnabled = false;
        autoAimMode = START_IN_AUTO_AIM;

        updateTurretTargetFromTeleOpGoal();

        robotHardware.reset_all();
        intake.intakeStop();
        outtake.stopOuttake();

        previousVelocityPose = follower.getPose();
        previousVelocityTime = getRuntime();
        estimatedFieldVelocityX = 0.0;
        estimatedFieldVelocityY = 0.0;
        estimatedFieldSpeed = 0.0;

        if (!autoAimMode) {
            turret.setRobotVelocityField(0.0, 0.0);
            turret.setForward();
        }

        telemetry.addLine("Blue TeleOp initialised");
        telemetry.addData("TeleOp Goal X", "%.2f", TELEOP_GOAL_X);
        telemetry.addData("TeleOp Goal Y", "%.2f", TELEOP_GOAL_Y);
        telemetry.addData("Turret Mode", autoAimMode ? "AUTO AIM" : "FORWARD");
        telemetry.addData("Turret Forward Servo", "%.3f", Turret.FORWARD_SERVO);
        telemetry.addData("Static TPS Mode", USE_STATIC_TARGET_TPS);
        telemetry.addData("Static Target TPS", "%.0f", STATIC_TARGET_TPS);
        telemetry.addData("Feed Drive Multiplier", "%.2f", FEED_DRIVE_MULTIPLIER);
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

        previousVelocityPose = follower.getPose();
        previousVelocityTime = getRuntime();
        estimatedFieldVelocityX = 0.0;
        estimatedFieldVelocityY = 0.0;
        estimatedFieldSpeed = 0.0;

        if (!autoAimMode) {
            turret.setRobotVelocityField(0.0, 0.0);
            turret.setForward();
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

        updateEstimatedFieldVelocity(currentPose);

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

        turret.setRobotVelocityField(0.0, 0.0);
        turret.setForward();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Drive controls
    //
    // gamepad1.left_stick_y:
    // - forward/back
    //
    // gamepad1.left_stick_x:
    // - strafe
    //
    // gamepad1.right_stick_x:
    // - turn
    //
    // gamepad1.right_bumper:
    // - intake only, blockers closed
    // - turn is reduced while collecting
    //
    // gamepad1.right_trigger:
    // - feed mode
    // - all drive movement is slowed while trigger is held
    // ─────────────────────────────────────────────────────────────────────────

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

    // ─────────────────────────────────────────────────────────────────────────
    // Turret controls
    //
    // gamepad1.left_bumper:
    // - toggle between AUTO AIM and FORWARD.
    //
    // gamepad1.dpad_up:
    // - force AUTO AIM.
    //
    // gamepad1.b:
    // - force FORWARD.
    // ─────────────────────────────────────────────────────────────────────────

    private void handleTurretModeControls() {
        boolean currentLeftBumper = gamepad1.left_bumper;
        boolean currentDpadUp = gamepad1.dpad_up;

        if (currentLeftBumper && !previousLeftBumper) {
            autoAimMode = !autoAimMode;

            if (!autoAimMode) {
                turret.setRobotVelocityField(0.0, 0.0);
                turret.setForward();
            }
        }

        if (currentDpadUp && !previousG1DpadUp) {
            autoAimMode = true;
            updateTurretTargetFromTeleOpGoal();

            Pose pose = follower.getPose();

            turret.aimTurret(
                    pose,
                    estimatedFieldVelocityX,
                    estimatedFieldVelocityY
            );
        }

        if (gamepad1.b) {
            autoAimMode = false;
            turret.setRobotVelocityField(0.0, 0.0);
            turret.setForward();
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
            turret.setRobotVelocityField(0.0, 0.0);
            turret.setForward();
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Shooter controls
    //
    // gamepad1.x / square:
    // - shooter on.
    //
    // gamepad1.y / triangle:
    // - everything off.
    // - hood reset.
    // - turret forward.
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

            turret.setRobotVelocityField(0.0, 0.0);
            turret.setForward();
        }

        previousSquare = currentSquare;
        previousTriangle = currentTriangle;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Pose reset
    //
    // gamepad1.dpad_right:
    // - reset live pose near blue goal.
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

            resetVelocityEstimate(resetPose);

            autoAimMode = true;
            updateTurretTargetFromTeleOpGoal();

            turret.aimTurret(
                    resetPose,
                    0.0,
                    0.0
            );
        }

        previousG1DpadRight = currentDpadRight;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Intake and feed controls
    //
    // gamepad1.right_trigger:
    // - feed only if shooter is already on.
    // - also activates drive slow mode through driveRobot().
    //
    // gamepad1.right_bumper:
    // - intake collect only.
    // - blockers stay closed.
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
    // Velocity estimation
    //
    // This replaces follower.getVelocity(), because your version returns a
    // double, not a Pose.
    //
    // This produces field X/Y velocity from pose changes.
    // Units should be inches per second if Pedro pose units are inches.
    // ─────────────────────────────────────────────────────────────────────────

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
        estimatedFieldSpeed = Math.hypot(estimatedFieldVelocityX, estimatedFieldVelocityY);

        previousVelocityPose = currentPose;
        previousVelocityTime = currentTime;
    }

    private void resetVelocityEstimate(Pose pose) {
        previousVelocityPose = pose;
        previousVelocityTime = getRuntime();

        estimatedFieldVelocityX = 0.0;
        estimatedFieldVelocityY = 0.0;
        estimatedFieldSpeed = 0.0;

        turret.setRobotVelocityField(0.0, 0.0);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // TeleOp target helpers
    //
    // TeleOp owns the goal target.
    // Turret target is updated from this TeleOp target.
    // Shooter distance also uses this TeleOp target.
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
        telemetry.addLine("Blue Internationals TeleOp - Velocity + Angular Compensation");

        telemetry.addLine("Controls");
        telemetry.addData("G1 Square / X", "Shooter ON");
        telemetry.addData("G1 Triangle / Y", "Everything OFF + hood reset");
        telemetry.addData("G1 Right Trigger", "Feed only + slow drive");
        telemetry.addData("G1 Right Bumper", "Intake only, blockers closed");
        telemetry.addData("G1 Left Bumper", "Toggle AUTO AIM / FORWARD");
        telemetry.addData("G1 Dpad Up", "Force AUTO AIM");
        telemetry.addData("G1 Dpad Right", "Reset live pose");
        telemetry.addData("G1 B", "Force FORWARD");

        telemetry.addLine("State");
        telemetry.addData("Shooter Enabled", shooterEnabled);
        telemetry.addData("Feed Requested", isFeedRequested());
        telemetry.addData("Feed Slow Mode", isFeedRequested());
        telemetry.addData("Turret Mode", autoAimMode ? "AUTO AIM" : "FORWARD");
        telemetry.addData("Collect Intake Active", isCollectIntakeActive());

        telemetry.addLine("TeleOp Tuneables");
        telemetry.addData("TeleOp Goal X", "%.2f", TELEOP_GOAL_X);
        telemetry.addData("TeleOp Goal Y", "%.2f", TELEOP_GOAL_Y);
        telemetry.addData("Reset X", "%.2f", TELEOP_RESET_X);
        telemetry.addData("Reset Y", "%.2f", TELEOP_RESET_Y);
        telemetry.addData("Reset Heading", "%.1f", TELEOP_RESET_HEADING_DEG);
        telemetry.addData("Start In Auto Aim", START_IN_AUTO_AIM);
        telemetry.addData("Drive Speed", "%.2f", DRIVE_SPEED);
        telemetry.addData("Intake Turn Multiplier", "%.2f", INTAKE_TURN_MULTIPLIER);
        telemetry.addData("Feed Drive Multiplier", "%.2f", FEED_DRIVE_MULTIPLIER);
        telemetry.addData("Shoot Trigger Threshold", "%.2f", SHOOT_TRIGGER_THRESHOLD);

        telemetry.addLine("Goal / Distance");
        telemetry.addData("Target X", "%.2f", getTeleOpGoalPose().getX());
        telemetry.addData("Target Y", "%.2f", getTeleOpGoalPose().getY());
        telemetry.addData("Distance CM", "%.1f", distCM);

        telemetry.addLine("Estimated Field Velocity");
        telemetry.addData("Velocity X", "%.2f", estimatedFieldVelocityX);
        telemetry.addData("Velocity Y", "%.2f", estimatedFieldVelocityY);
        telemetry.addData("Speed", "%.2f", estimatedFieldSpeed);
        telemetry.addData("Angular Velocity", "%.4f", follower.getAngularVelocity());

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

        telemetry.addLine("Turret Compensation");
        telemetry.addData("Turret Forward Servo", "%.3f", Turret.FORWARD_SERVO);
        telemetry.addData("Turret Target X", "%.2f", Turret.targetPose.getX());
        telemetry.addData("Turret Target Y", "%.2f", Turret.targetPose.getY());
        telemetry.addData("Lead X", "%.2f", Turret.leadX);
        telemetry.addData("Lead Y", "%.2f", Turret.leadY);
        telemetry.addData("Desired Servo", "%.3f", Turret.desiredServo);
        telemetry.addData("Current Servo", "%.3f", Turret.currentServo);
        telemetry.addData("Servo Error", "%.4f", Turret.servoError);

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