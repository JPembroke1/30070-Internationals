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
@TeleOp(name = "01 - Far Zone Shooter Tuning", group = "Testing")
public class FarZoneShooterTuningTeleOp extends OpMode {

    private final Pose startPose = new Pose(56, 8.5, Math.toRadians(180));

    private Follower follower;
    private Intake intake;
    private Outtake outtake;
    private Turret turret;
    private RobotHardware robotHardware;

    private List<LynxModule> allHubs;

    public static double BLUE_GOAL_X = 6.0;
    public static double BLUE_GOAL_Y = 138.0;

    public static double TELEOP_GOAL_X = 6.0;
    public static double TELEOP_GOAL_Y = 138.0;

    public static double FAR_ZONE_RESET_X = 56.0;
    public static double FAR_ZONE_RESET_Y = 8.5;
    public static double FAR_ZONE_RESET_HEADING_DEG = 180.0;

    public static boolean START_IN_AUTO_AIM = true;

    public static double AIM_OFFSET_STEP = 0.005;
    public static double AIM_OFFSET_MIN = -0.20;
    public static double AIM_OFFSET_MAX = 0.20;

    public static double DRIVE_SPEED = 1.0;
    public static double INTAKE_TURN_MULTIPLIER = 0.45;
    public static double FEED_DRIVE_MULTIPLIER = 0.50;

    public static double FAR_ZONE_TARGET_TPS = 2760.0;
    public static double TPS_STEP_SMALL = 25.0;
    public static double TPS_STEP_LARGE = 100.0;
    public static double MIN_TARGET_TPS = 0.0;
    public static double MAX_TARGET_TPS = 4000.0;

    public static double SHOOT_TRIGGER_THRESHOLD = 0.2;

    public static double SHOOT_INTAKE_LEFT_POWER = 0.3;
    public static double SHOOT_INTAKE_RIGHT_POWER = 0.3;

    public static double COLLECT_INTAKE_LEFT_POWER = 1.0;
    public static double COLLECT_INTAKE_RIGHT_POWER = 1.0;

    public static boolean USE_POSE_DELTA_VELOCITY = true;

    private String allianceLabel = "FAR ZONE";

    private boolean shooterEnabled = false;
    private boolean autoAimMode = true;

    private boolean previousG1DpadUp = false;
    private boolean previousG1DpadRight = false;
    private boolean previousLeftBumper = false;

    private boolean previousG2DpadLeft = false;
    private boolean previousG2DpadRight = false;
    private boolean previousG2Square = false;

    private boolean previousSquare = false;
    private boolean previousTriangle = false;
    private boolean previousG2Triangle = false;

    private boolean previousG2DpadUp = false;
    private boolean previousG2DpadDown = false;
    private boolean previousG2LeftBumper = false;
    private boolean previousG2RightBumper = false;

    private Pose previousVelocityPose = null;
    private double previousVelocityTime = 0.0;

    private double estimatedFieldVelocityX = 0.0;
    private double estimatedFieldVelocityY = 0.0;
    private double estimatedFieldSpeed = 0.0;

    @Override
    public void init() {
        configureBulkCaching();
        clearBulkCache();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        PoseStorage.setPose(startPose);

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

        telemetry.addLine("Far Zone Shooter Tuning Initialised");
        telemetry.addData("Start X", "%.2f", startPose.getX());
        telemetry.addData("Start Y", "%.2f", startPose.getY());
        telemetry.addData("Start Heading", "%.1f", Math.toDegrees(startPose.getHeading()));
        telemetry.addData("Goal X", "%.2f", TELEOP_GOAL_X);
        telemetry.addData("Goal Y", "%.2f", TELEOP_GOAL_Y);
        telemetry.addData("Far Zone Requested TPS", "%.1f", FAR_ZONE_TARGET_TPS);
        telemetry.addData("Outtake Far Zone Min", "%.1f", Outtake.farZoneMinTargetTPS);
        telemetry.addData("Outtake Far Zone Max", "%.1f", Outtake.farZoneMaxTargetTPS);
        telemetry.addData("Far Hood Slope", "%.5f", RobotHardware.farZoneHoodSlope);
        telemetry.addData("Far Hood Intercept", "%.3f", RobotHardware.farZoneHoodIntercept);
        telemetry.addData("Far Hood Velocity Factor", "%.3f", RobotHardware.farZoneHoodVelocityFactor);
        telemetry.addData("Bulk Caching", "MANUAL");
        telemetry.update();
    }

    @Override
    public void start() {
        clearBulkCache();

        follower.startTeleopDrive();

        follower.setPose(startPose);
        PoseStorage.setPose(startPose);

        shooterEnabled = false;
        autoAimMode = START_IN_AUTO_AIM;

        FAR_ZONE_TARGET_TPS = clamp(FAR_ZONE_TARGET_TPS, MIN_TARGET_TPS, MAX_TARGET_TPS);

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
        handleShooterTuningControls();
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
            FAR_ZONE_TARGET_TPS = clamp(FAR_ZONE_TARGET_TPS, MIN_TARGET_TPS, MAX_TARGET_TPS);
            outtake.setFarZoneTargetTPS(FAR_ZONE_TARGET_TPS);
        } else {
            outtake.stopOuttake();
        }

        outtake.updatePIDF();

        robotHardware.farZoneHoodRegression(distCM);

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

    private void handleShooterTuningControls() {
        boolean currentG2DpadUp = gamepad2.dpad_up;
        boolean currentG2DpadDown = gamepad2.dpad_down;
        boolean currentG2LeftBumper = gamepad2.left_bumper;
        boolean currentG2RightBumper = gamepad2.right_bumper;

        if (currentG2DpadUp && !previousG2DpadUp) {
            FAR_ZONE_TARGET_TPS += TPS_STEP_SMALL;
            FAR_ZONE_TARGET_TPS = clamp(FAR_ZONE_TARGET_TPS, MIN_TARGET_TPS, MAX_TARGET_TPS);
        }

        if (currentG2DpadDown && !previousG2DpadDown) {
            FAR_ZONE_TARGET_TPS -= TPS_STEP_SMALL;
            FAR_ZONE_TARGET_TPS = clamp(FAR_ZONE_TARGET_TPS, MIN_TARGET_TPS, MAX_TARGET_TPS);
        }

        if (currentG2RightBumper && !previousG2RightBumper) {
            FAR_ZONE_TARGET_TPS += TPS_STEP_LARGE;
            FAR_ZONE_TARGET_TPS = clamp(FAR_ZONE_TARGET_TPS, MIN_TARGET_TPS, MAX_TARGET_TPS);
        }

        if (currentG2LeftBumper && !previousG2LeftBumper) {
            FAR_ZONE_TARGET_TPS -= TPS_STEP_LARGE;
            FAR_ZONE_TARGET_TPS = clamp(FAR_ZONE_TARGET_TPS, MIN_TARGET_TPS, MAX_TARGET_TPS);
        }

        previousG2DpadUp = currentG2DpadUp;
        previousG2DpadDown = currentG2DpadDown;
        previousG2LeftBumper = currentG2LeftBumper;
        previousG2RightBumper = currentG2RightBumper;
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
            Pose resetPose = getFarZoneResetPose();

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

    private Pose getFarZoneResetPose() {
        return new Pose(
                FAR_ZONE_RESET_X,
                FAR_ZONE_RESET_Y,
                Math.toRadians(FAR_ZONE_RESET_HEADING_DEG)
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
        String farZoneMode = Outtake.usingFarZoneTarget ? "FAR" : "NORMAL";

        telemetry.addLine(
                allianceLabel +
                        " | SHOOTER:" + shooterState +
                        " | " + readyState +
                        " | " + turretState +
                        " | " + feedState
        );

        telemetry.addLine(
                "DIST:" + format("%.0f", distCM) +
                        "cm | MODE:" + farZoneMode
        );

        telemetry.addLine(
                "REQUEST:" + format("%.0f", Outtake.requestedTarget) +
                        " | TARGET:" + format("%.0f", Outtake.target) +
                        " | CURRENT:" + format("%.0f", Outtake.currentTPS)
        );

        telemetry.addLine(
                "HOOD RAW:" + format("%.3f", RobotHardware.lastRawHoodTarget) +
                        " | CLIPPED:" + format("%.3f", RobotHardware.lastClippedHoodTarget) +
                        " | CMD:" + format("%.3f", RobotHardware.lastCommandedHood)
        );

        telemetry.addLine(
                "FAR HOOD SLOPE:" + format("%.5f", RobotHardware.farZoneHoodSlope) +
                        " | INT:" + format("%.3f", RobotHardware.farZoneHoodIntercept)
        );

        telemetry.addLine(
                "VEL COMP FACTOR:" + format("%.3f", RobotHardware.farZoneHoodVelocityFactor) +
                        " | OFFSET:" + format("%.3f", Turret.AIM_OFFSET)
        );

        telemetry.addLine(
                "TPS STEP:" + format("%.0f", TPS_STEP_SMALL) +
                        "/" + format("%.0f", TPS_STEP_LARGE)
        );

        telemetry.addLine(
                "POSE X:" + format("%.1f", follower.getPose().getX()) +
                        " Y:" + format("%.1f", follower.getPose().getY()) +
                        " H:" + format("%.1f", Math.toDegrees(follower.getPose().getHeading()))
        );

        telemetry.addLine("G1 X: shooter on | G1 Y/G2 Y: reset | G1 RT: feed");
        telemetry.addLine("G2 D-pad up/down: TPS small | G2 bumpers: TPS large");

        telemetry.update();
    }
}