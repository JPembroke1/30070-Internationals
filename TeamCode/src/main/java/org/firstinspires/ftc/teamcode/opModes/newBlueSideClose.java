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
class BlueTeleOP extends OpMode {

    private Follower follower;
    private Intake intake;
    private Outtake outtake;
    private Turret turret;
    private RobotHardware robotHardware;

    // ─────────────────────────────────────────────────────────
    // TeleOp target
    // ─────────────────────────────────────────────────────────

    public static double TELEOP_GOAL_X = 6;
    public static double TELEOP_GOAL_Y = 138;

    public static double RESET_X = 24;
    public static double RESET_Y = 138;
    public static double RESET_HEADING = 177;

    // ─────────────────────────────────────────────────────────
    // Turret
    // ─────────────────────────────────────────────────────────

    public static boolean START_IN_AUTO = true;
    private boolean autoAimMode = true;
    private boolean prevLB = false;

    // ─────────────────────────────────────────────────────────
    // Drive
    // ─────────────────────────────────────────────────────────

    public static double DRIVE_SPEED = 1.0;
    public static double INTAKE_TURN_MULTIPLIER = 0.45;

    // ✅ NEW
    public static double FEED_DRIVE_MULTIPLIER = 0.5;

    // ─────────────────────────────────────────────────────────
    // Shooter
    // ─────────────────────────────────────────────────────────

    public static boolean USE_STATIC = false;
    public static double STATIC_TPS = 1500;

    private boolean shooterEnabled = false;
    private boolean prevX = false;
    private boolean prevY = false;

    // ─────────────────────────────────────────────────────────
    // Intake
    // ─────────────────────────────────────────────────────────

    public static double TRIGGER_THRESHOLD = 0.2;

    public static double SHOOT_L = 0.7;
    public static double SHOOT_R = 0.9;

    public static double COLLECT_L = 1.0;
    public static double COLLECT_R = 1.0;

    // ─────────────────────────────────────────────────────────
    // Init
    // ─────────────────────────────────────────────────────────

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(new Pose(60, 80, Math.toRadians(177)));

        outtake = new Outtake();
        intake = new Intake();
        turret = new Turret();
        robotHardware = new RobotHardware(hardwareMap);

        outtake.init(hardwareMap);
        intake.init(hardwareMap);
        turret.init(hardwareMap);

        autoAimMode = START_IN_AUTO;

        robotHardware.reset_all();
        intake.intakeStop();
        outtake.stopOuttake();
        turret.setForward();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        handleControls();

        follower.update();

        Pose pose = follower.getPose();
        PoseStorage.currentPose = pose;

        turret.setTargetPose(getGoalPose());

        if (autoAimMode) {
            turret.aimTurret(pose);
        } else {
            turret.setForward();
        }

        double dist = distanceToGoalCM();

        if (shooterEnabled) {
            if (USE_STATIC) {
                outtake.setTargetTPS(STATIC_TPS);
            } else {
                outtake.linearRegression(dist);
            }
            robotHardware.linearHoodRegression(dist);
        } else {
            outtake.stopOuttake();
        }

        outtake.updatePIDF();

        intake.update();

        drive();

        telemetry.addData("Mode", autoAimMode ? "AUTO" : "FORWARD");
        telemetry.addData("Feed", isFeed());
        telemetry.addData("Shooter", shooterEnabled);
        telemetry.update();
    }

    // ─────────────────────────────────────────────────────────
    // Controls
    // ─────────────────────────────────────────────────────────

    private void handleControls() {

        boolean lb = gamepad1.left_bumper;
        if (lb && !prevLB) autoAimMode = !autoAimMode;
        prevLB = lb;

        if (gamepad1.dpad_up) autoAimMode = true;
        if (gamepad1.b) autoAimMode = false;

        boolean x = gamepad1.x;
        boolean y = gamepad1.y;

        if (x && !prevX) shooterEnabled = true;

        if (y && !prevY) {
            shooterEnabled = false;
            autoAimMode = false;
            outtake.stopOuttake();
            intake.intakeStop();
            robotHardware.reset_all();
            turret.setForward();
        }

        prevX = x;
        prevY = y;

        if (gamepad1.dpad_right) {
            Pose reset = new Pose(
                    RESET_X,
                    RESET_Y,
                    Math.toRadians(RESET_HEADING)
            );

            follower.setPose(reset);
            PoseStorage.currentPose = reset;

            autoAimMode = true;
        }
    }

    // ─────────────────────────────────────────────────────────
    // Drive (UPDATED WITH SLOW MODE)
    // ─────────────────────────────────────────────────────────

    private void drive() {
        double f = -gamepad1.left_stick_y * DRIVE_SPEED;
        double s = -gamepad1.left_stick_x * DRIVE_SPEED;
        double t = -gamepad1.right_stick_x * DRIVE_SPEED;

        if (isCollect()) {
            t *= INTAKE_TURN_MULTIPLIER;
        }

        // ✅ FEED SLOW MODE
        if (isFeed()) {
            f *= FEED_DRIVE_MULTIPLIER;
            s *= FEED_DRIVE_MULTIPLIER;
            t *= FEED_DRIVE_MULTIPLIER;
        }

        follower.setTeleOpDrive(f, s, t);
    }

    // ─────────────────────────────────────────────────────────
    // Intake & Feed
    // ─────────────────────────────────────────────────────────

    private void intakeControl() {

        if (isFeed() && shooterEnabled) {
            robotHardware.release();
            intake.intake(SHOOT_L, SHOOT_R);
            return;
        }

        if (isCollect()) {
            robotHardware.block();
            intake.intake(COLLECT_L, COLLECT_R);
            return;
        }

        robotHardware.block();
        intake.intakeStop();
    }

    private boolean isCollect() {
        return gamepad1.right_bumper;
    }

    private boolean isFeed() {
        return gamepad1.right_trigger > TRIGGER_THRESHOLD;
    }

    // ─────────────────────────────────────────────────────────
    // Goal
    // ─────────────────────────────────────────────────────────

    private Pose getGoalPose() {
        return new Pose(TELEOP_GOAL_X, TELEOP_GOAL_Y, 0);
    }

    private double distanceToGoalCM() {
        Pose p = follower.getPose();
        Pose g = getGoalPose();

        double dx = g.getX() - p.getX();
        double dy = g.getY() - p.getY();

        return Math.hypot(dx, dy) * 2.54;
    }
}
