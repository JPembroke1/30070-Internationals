package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.opModes.subClasses.Intake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Outtake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.RobotHardware;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Configurable
@Autonomous(name = "Blue Side Close Comp", group = "Blue")
public class newBlueSideClose extends OpMode {

    // ─────────────────────────────────────────────────────────────────────────
    // Subsystems
    // ─────────────────────────────────────────────────────────────────────────

    private Follower follower;
    private Intake intake;
    private Outtake outtake;
    private Turret turret;
    private RobotHardware robotHardware;

    // ─────────────────────────────────────────────────────────────────────────
    // Goal target
    //
    // These values are the single source of truth for:
    // - turret target
    // - shooter regression distance
    // - hood regression distance
    //
    // Field coordinates are in inches.
    // ─────────────────────────────────────────────────────────────────────────

    public static double GOAL_X = 6.0;
    public static double GOAL_Y = 138.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Shooting and collection timing
    //
    // Tuning notes:
    // - Increase SHOOT_SETTLE_SECONDS if the first shot is weak.
    // - Increase SHOOT_FEED_SECONDS if not all balls feed.
    // - Increase GATE_COLLECT_SECONDS if stack/gate collection is unreliable.
    // ─────────────────────────────────────────────────────────────────────────

    public static double SHOOT_SETTLE_SECONDS = 0.35;
    public static double SHOOT_FEED_SECONDS = 2.0;
    public static double GATE_COLLECT_SECONDS = 2.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Intake powers
    //
    // Shooting feed power can be gentler than collection if balls jam or bounce.
    // ─────────────────────────────────────────────────────────────────────────

    public static double SHOOT_INTAKE_LEFT_POWER = 1.0;
    public static double SHOOT_INTAKE_RIGHT_POWER = 1.0;

    public static double COLLECT_INTAKE_LEFT_POWER = 1.0;
    public static double COLLECT_INTAKE_RIGHT_POWER = 1.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Path timeouts
    //
    // Tuning notes:
    // - Reduce if auto waits too long after paths.
    // - Increase if paths are being cut short before the robot reaches position.
    // ─────────────────────────────────────────────────────────────────────────

    public static double TIMEOUT_PATH_1 = 4.0;
    public static double TIMEOUT_PATH_2 = 3.0;
    public static double TIMEOUT_PATH_3 = 3.0;
    public static double TIMEOUT_PATH_4 = 4.0;
    public static double TIMEOUT_PATH_5 = 2.0;
    public static double TIMEOUT_PATH_6 = 4.0;
    public static double TIMEOUT_PATH_7 = 3.0;
    public static double TIMEOUT_PATH_8 = 3.0;
    public static double TIMEOUT_PATH_9 = 3.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Shooter readiness
    //
    // TPS_READY_THRESHOLD:
    // - 0.95 means shooter is ready at 95% of target TPS.
    //
    // Example:
    // - 1500 TPS target × 0.95 = 1425 TPS ready threshold.
    // ─────────────────────────────────────────────────────────────────────────

    public static double TPS_SPINUP_TIMEOUT = 2.0;
    public static double TPS_READY_THRESHOLD = 0.95;

    private double tpsWaitStartTime = 0.0;
    private boolean tpsTimeoutFired = false;
    private boolean outtakeEnabled = true;

    // ─────────────────────────────────────────────────────────────────────────
    // Velocity estimation
    //
    // Your follower.getVelocity() returns a primitive double, so this autonomous
    // estimates field X/Y velocity from pose change each loop.
    //
    // Since Pedro pose X/Y are field coordinates in inches, this gives estimated
    // field-relative velocity in inches/sec.
    //
    // VELOCITY_SMOOTHING_ALPHA:
    // - 1.0 = no smoothing.
    // - lower = smoother but more lag.
    // - 0.35 is a reasonable starting point.
    // ─────────────────────────────────────────────────────────────────────────

    public static boolean USE_POSE_DELTA_VELOCITY = true;
    public static double VELOCITY_SMOOTHING_ALPHA = 0.35;

    private Pose previousVelocityPose = null;
    private double previousVelocityTime = 0.0;

    private double rawVelocityX = 0.0;
    private double rawVelocityY = 0.0;

    private double estimatedVelocityX = 0.0;
    private double estimatedVelocityY = 0.0;

    private double velocityDt = 0.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Field poses
    // ─────────────────────────────────────────────────────────────────────────

    private final Pose startPose = new Pose(21, 121, Math.toRadians(143));
    private final Pose Shoot = new Pose(60, 80, Math.toRadians(177));
    private final Pose Stack_1 = new Pose(17, 82, Math.toRadians(177));
    private final Pose Stack_2 = new Pose(40, 56, Math.toRadians(170));
    private final Pose EatStack_2 = new Pose(15, 60, Math.toRadians(170));
    private final Pose OverFlow = new Pose(13, 60, Math.toRadians(150));
    private final Pose End = new Pose(50, 70, Math.toRadians(0));

    // ─────────────────────────────────────────────────────────────────────────
    // Path chains
    // ─────────────────────────────────────────────────────────────────────────

    private PathChain pathToPos1;
    private PathChain pathToPos2;
    private PathChain pathToPos3;
    private PathChain pathToPos4;
    private PathChain pathToPos5;
    private PathChain pathToPos6;
    private PathChain pathToPos7;
    private PathChain pathToPos8;
    private PathChain pathToPos9;

    // ─────────────────────────────────────────────────────────────────────────
    // Init
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        outtake = new Outtake();
        intake = new Intake();
        turret = new Turret();
        robotHardware = new RobotHardware(hardwareMap);

        outtake.init(hardwareMap);
        intake.init(hardwareMap);

        // Uses follower angular velocity for optional angular compensation in Turret.
        turret.init(hardwareMap, follower::getAngularVelocity);

        updateGoalTarget();

        robotHardware.reset_all();
        intake.intakeStop();
        outtake.stopOuttake();

        turret.clearRobotVelocity();
        turret.setForward();

        resetVelocityEstimator(startPose);

        telemetry.addLine("Blue Side Close Comp initialised");
        telemetry.addData("Goal X", "%.2f", GOAL_X);
        telemetry.addData("Goal Y", "%.2f", GOAL_Y);
        telemetry.addData("Turret Forward Servo", "%.3f", Turret.FORWARD_SERVO);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        updateGoalTarget();

        double distCM = distanceToGoalCM();

        outtake.linearRegression(distCM);
        robotHardware.linearHoodRegression(distCM);

        telemetry.addLine("Blue Side Close Comp init loop");
        telemetry.addData("Goal X", "%.2f", GOAL_X);
        telemetry.addData("Goal Y", "%.2f", GOAL_Y);
        telemetry.addData("Distance CM", "%.1f", distCM);
        telemetry.addData("Target TPS", "%.0f", Outtake.target);
        telemetry.addData("Hood Command", "%.3f", RobotHardware.lastCommandedHood);
        telemetry.update();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Start command schedule
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void start() {
        Scheduler.reset();

        robotHardware.reset_all();

        turret.clearRobotVelocity();
        turret.setForward();

        PoseStorage.currentPose = startPose;

        resetVelocityEstimator(startPose);
        updateGoalTarget();

        outtakeEnabled = true;
        tpsTimeoutFired = false;

        stopIntakeAndBlock();

        schedule(
                infinite(this::robotPeriodic),

                sequential(
                        followWithTimeout(pathToPos1, TIMEOUT_PATH_1),
                        shootCycle(),

                        followWithTimeout(pathToPos2, TIMEOUT_PATH_2),
                        followWithTimeout(pathToPos3, TIMEOUT_PATH_3),
                        shootCycle(),

                        followWithTimeout(pathToPos4, TIMEOUT_PATH_4),
                        followCollectWithTimeout(pathToPos5, TIMEOUT_PATH_5),
                        gateCollectWait(),

                        followWithTimeout(pathToPos6, TIMEOUT_PATH_6),
                        shootCycle(),

                        followWithTimeout(pathToPos7, TIMEOUT_PATH_7),
                        followWithTimeout(pathToPos8, TIMEOUT_PATH_8),
                        shootCycle(),

                        followWithTimeout(pathToPos9, TIMEOUT_PATH_9),
                        instant(this::finishAuto)
                )
        );
    }

    @Override
    public void loop() {
        Scheduler.execute();

        telemetry.addLine("Blue Side Close Comp");
        telemetry.addData("Outtake Enabled", outtakeEnabled);
        telemetry.addData("TPS Timeout Fired", tpsTimeoutFired);

        telemetry.addLine("Velocity Estimate");
        telemetry.addData("Raw Velocity X", "%.2f", rawVelocityX);
        telemetry.addData("Raw Velocity Y", "%.2f", rawVelocityY);
        telemetry.addData("Estimated Velocity X", "%.2f", estimatedVelocityX);
        telemetry.addData("Estimated Velocity Y", "%.2f", estimatedVelocityY);
        telemetry.addData("Velocity dt", "%.3f", velocityDt);

        telemetry.addLine("Turret Compensation");
        telemetry.addData("Velocity Comp Active", Turret.velocityCompensationActive);
        telemetry.addData("Velocity Auto Disabled", Turret.velocityCompensationAutoDisabled);
        telemetry.addData("Robot Speed", "%.2f", Turret.robotSpeed);
        telemetry.addData("Lead X", "%.2f", Turret.leadX);
        telemetry.addData("Lead Y", "%.2f", Turret.leadY);
        telemetry.addData("Lead Mag", "%.2f", Turret.leadMagnitude);
        telemetry.addData("Comp Target X", "%.2f", Turret.compensatedTargetX);
        telemetry.addData("Comp Target Y", "%.2f", Turret.compensatedTargetY);

        telemetry.addLine("Shooter");
        telemetry.addData("Target TPS", "%.0f", Outtake.target);
        telemetry.addData("Current TPS", "%.0f", Outtake.currentTPS);
        telemetry.addData("At Speed", outtake.isAtSpeed(TPS_READY_THRESHOLD));

        telemetry.update();
    }

    @Override
    public void stop() {
        PoseStorage.currentPose = follower.getPose();

        outtakeEnabled = false;

        outtake.stopOuttake();
        outtake.updatePIDF();

        stopIntakeAndBlock();
        robotHardware.reset_all();

        turret.clearRobotVelocity();
        turret.setForward();

        Scheduler.reset();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Path building
    // ─────────────────────────────────────────────────────────────────────────

    private void buildPaths() {
        pathToPos1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, Shoot))
                .setLinearHeadingInterpolation(startPose.getHeading(), Shoot.getHeading())
                .build();

        pathToPos2 = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, Stack_1))
                .setLinearHeadingInterpolation(Shoot.getHeading(), Stack_1.getHeading())
                .build();

        pathToPos3 = follower.pathBuilder()
                .addPath(new BezierLine(Stack_1, Shoot))
                .setLinearHeadingInterpolation(Stack_1.getHeading(), Shoot.getHeading())
                .build();

        pathToPos4 = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, Stack_2))
                .setLinearHeadingInterpolation(Shoot.getHeading(), Stack_2.getHeading())
                .build();

        pathToPos5 = follower.pathBuilder()
                .addPath(new BezierLine(Stack_2, EatStack_2))
                .setLinearHeadingInterpolation(Stack_2.getHeading(), EatStack_2.getHeading())
                .build();

        pathToPos6 = follower.pathBuilder()
                .addPath(new BezierLine(EatStack_2, Shoot))
                .setLinearHeadingInterpolation(EatStack_2.getHeading(), Shoot.getHeading())
                .build();

        pathToPos7 = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, OverFlow))
                .setLinearHeadingInterpolation(Shoot.getHeading(), OverFlow.getHeading())
                .build();

        pathToPos8 = follower.pathBuilder()
                .addPath(new BezierLine(OverFlow, Shoot))
                .setLinearHeadingInterpolation(OverFlow.getHeading(), Shoot.getHeading())
                .build();

        pathToPos9 = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, End))
                .setLinearHeadingInterpolation(Shoot.getHeading(), End.getHeading())
                .build();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Ivy command helpers
    // ─────────────────────────────────────────────────────────────────────────

    private Command waitSeconds(double seconds) {
        return waitMs(seconds * 1000.0);
    }

    private Command followWithTimeout(PathChain path, double timeoutSeconds) {
        return race(
                follow(follower, path),
                waitSeconds(timeoutSeconds)
        );
    }

    private Command followCollectWithTimeout(PathChain path, double timeoutSeconds) {
        return race(
                followWithTimeout(path, timeoutSeconds),
                infinite(this::runGateCollect)
        );
    }

    private Command gateCollectWait() {
        return sequential(
                race(
                        waitSeconds(GATE_COLLECT_SECONDS),
                        infinite(this::runGateCollect)
                ),
                instant(this::stopIntakeAndBlock)
        );
    }

    private Command shootCycle() {
        return sequential(
                instant(() -> {
                    startTpsWait();
                    stopIntakeAndBlock();
                }),

                race(
                        waitUntil(this::shooterReadyOrTimedOut),
                        infinite(this::stopIntakeAndBlock)
                ),

                race(
                        waitSeconds(SHOOT_SETTLE_SECONDS),
                        infinite(this::stopIntakeAndBlock)
                ),

                race(
                        waitSeconds(SHOOT_FEED_SECONDS),
                        infinite(this::runShootFeed)
                ),

                instant(this::stopIntakeAndBlock)
        );
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Continuous robot update
    //
    // Keeps:
    // - Pedro updated
    // - pose stored
    // - velocity estimated
    // - turret aiming
    // - shooter running
    // - hood regression active
    // ─────────────────────────────────────────────────────────────────────────

    private void robotPeriodic() {
        follower.update();

        Pose currentPose = follower.getPose();
        PoseStorage.currentPose = currentPose;

        updateEstimatedVelocity(currentPose);

        updateGoalTarget();

        turret.aimTurret(
                currentPose,
                estimatedVelocityX,
                estimatedVelocityY
        );

        double distCM = distanceToGoalCM();

        if (outtakeEnabled) {
            outtake.linearRegression(distCM);
            robotHardware.linearHoodRegression(distCM);
        } else {
            outtake.stopOuttake();
        }

        outtake.updatePIDF();
    }

    private void finishAuto() {
        outtakeEnabled = false;

        outtake.stopOuttake();
        outtake.updatePIDF();

        stopIntakeAndBlock();
        robotHardware.reset_all();

        turret.clearRobotVelocity();
        turret.setForward();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Shooter readiness helpers
    // ─────────────────────────────────────────────────────────────────────────

    private void startTpsWait() {
        tpsWaitStartTime = getRuntime();
        tpsTimeoutFired = false;
    }

    private boolean hasSpinUpTimedOut() {
        return (getRuntime() - tpsWaitStartTime) > TPS_SPINUP_TIMEOUT;
    }

    private boolean isShooterAtSpeed() {
        return outtake.isAtSpeed(TPS_READY_THRESHOLD);
    }

    private boolean shooterReadyOrTimedOut() {
        if (isShooterAtSpeed()) {
            return true;
        }

        if (hasSpinUpTimedOut()) {
            tpsTimeoutFired = true;
            return true;
        }

        return false;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Intake and gate helpers
    // ─────────────────────────────────────────────────────────────────────────

    private void stopIntakeAndBlock() {
        intake.intakeStop();
        robotHardware.block();
    }

    private void runShootFeed() {
        robotHardware.release();
        intake.intake(SHOOT_INTAKE_LEFT_POWER, SHOOT_INTAKE_RIGHT_POWER);
    }

    private void runGateCollect() {
        robotHardware.release();
        intake.intake(COLLECT_INTAKE_LEFT_POWER, COLLECT_INTAKE_RIGHT_POWER);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Velocity estimation
    //
    // Uses pose delta:
    // velocity = change in field position / change in time
    //
    // This avoids follower.getVelocity(), because your current Follower version
    // returns a primitive double instead of X/Y velocity.
    // ─────────────────────────────────────────────────────────────────────────

    private void resetVelocityEstimator(Pose pose) {
        previousVelocityPose = pose;
        previousVelocityTime = getRuntime();

        rawVelocityX = 0.0;
        rawVelocityY = 0.0;

        estimatedVelocityX = 0.0;
        estimatedVelocityY = 0.0;

        velocityDt = 0.0;

        turret.clearRobotVelocity();
    }

    private void updateEstimatedVelocity(Pose currentPose) {
        if (!USE_POSE_DELTA_VELOCITY || currentPose == null) {
            rawVelocityX = 0.0;
            rawVelocityY = 0.0;

            estimatedVelocityX = 0.0;
            estimatedVelocityY = 0.0;

            velocityDt = 0.0;

            turret.clearRobotVelocity();
            return;
        }

        double currentTime = getRuntime();

        if (previousVelocityPose == null) {
            resetVelocityEstimator(currentPose);
            return;
        }

        velocityDt = currentTime - previousVelocityTime;

        if (velocityDt <= 0.001) {
            return;
        }

        rawVelocityX = (currentPose.getX() - previousVelocityPose.getX()) / velocityDt;
        rawVelocityY = (currentPose.getY() - previousVelocityPose.getY()) / velocityDt;

        double alpha = clamp(VELOCITY_SMOOTHING_ALPHA, 0.0, 1.0);

        estimatedVelocityX = (alpha * rawVelocityX) + ((1.0 - alpha) * estimatedVelocityX);
        estimatedVelocityY = (alpha * rawVelocityY) + ((1.0 - alpha) * estimatedVelocityY);

        previousVelocityPose = currentPose;
        previousVelocityTime = currentTime;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Goal and distance helpers
    // ─────────────────────────────────────────────────────────────────────────

    private Pose getGoalPose() {
        return new Pose(
                GOAL_X,
                GOAL_Y,
                0
        );
    }

    private void updateGoalTarget() {
        turret.setTargetPose(getGoalPose());
    }

    private double distanceToGoalCM() {
        Pose robotPose = follower.getPose();
        Pose goalPose = getGoalPose();

        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();

        return Math.hypot(dx, dy) * 2.54;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Utility
    // ─────────────────────────────────────────────────────────────────────────

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}