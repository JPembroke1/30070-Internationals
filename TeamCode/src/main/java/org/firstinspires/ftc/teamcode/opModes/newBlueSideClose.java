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


    public static double GOAL_X = 6.0;
    public static double GOAL_Y = 138.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Shooting and collection timing
    // ─────────────────────────────────────────────────────────────────────────

    public static double SHOOT_SETTLE_SECONDS = 0.5;
    public static double SHOOT_FEED_SECONDS = 1.0;
    public static double GATE_COLLECT_SECONDS = 2.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Intake powers
    // ─────────────────────────────────────────────────────────────────────────

    public static double SHOOT_INTAKE_LEFT_POWER = 0.5;
    public static double SHOOT_INTAKE_RIGHT_POWER = 0.5;

    public static double COLLECT_INTAKE_LEFT_POWER = 1.0;
    public static double COLLECT_INTAKE_RIGHT_POWER = 1.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Path timeouts
    // ─────────────────────────────────────────────────────────────────────────

    public static double TIMEOUT_PATH_1 = 3.0;
    public static double TIMEOUT_PATH_2 = 3.0;
    public static double TIMEOUT_PATH_3 = 3.0;
    public static double TIMEOUT_PATH_4 = 3.0;
    public static double TIMEOUT_PATH_5 = 2.0;
    public static double TIMEOUT_PATH_6 = 3.0;
    public static double TIMEOUT_PATH_7 = 3.0;
    public static double TIMEOUT_PATH_8 = 3.0;
    public static double TIMEOUT_PATH_9 = 3.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Shooter readiness
    //
    // If shooter does not reach speed before timeout, auto feeds anyway.
    // ─────────────────────────────────────────────────────────────────────────

    public static double TPS_SPINUP_TIMEOUT = 2.0;
    public static double TPS_READY_THRESHOLD = 0.95;

    private double tpsWaitStartTime = 0.0;
    private boolean tpsTimeoutFired = false;
    private boolean outtakeEnabled = true;

    // ─────────────────────────────────────────────────────────────────────────
    // Velocity estimation
    //
    // Uses pose delta because this Pedro version does not return X/Y velocity.
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

    private double currentDistanceCM = 0.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Field poses
    // ─────────────────────────────────────────────────────────────────────────

    private final Pose startPose = new Pose(22, 124, Math.toRadians(145));
    private final Pose shootPose = new Pose(60, 86, Math.toRadians(180));

    private final Pose stack1Pose = new Pose(13, 86, Math.toRadians(180));

    private final Pose stack2Pose = new Pose(40, 60, Math.toRadians(180));
    private final Pose eatStack2Pose = new Pose(15, 60, Math.toRadians(180));

    private final Pose overflowPose = new Pose(15, 58, Math.toRadians(145));
    private final Pose endPose = new Pose(50, 70, Math.toRadians(180));

    // ─────────────────────────────────────────────────────────────────────────
    // Path chains
    // ─────────────────────────────────────────────────────────────────────────

    private PathChain pathToShoot;
    private PathChain pathToStack1;
    private PathChain pathReturnFromStack1;
    private PathChain pathToStack2;
    private PathChain pathToEatStack2;
    private PathChain pathReturnFromStack2;
    private PathChain pathToOverflow;
    private PathChain pathReturnFromOverflow;
    private PathChain pathToEnd;

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
        turret.init(hardwareMap);

        updateGoalTarget();

        robotHardware.reset_all();
        intake.intakeStop();
        outtake.stopOuttake();
        turret.setForward();

        resetVelocityEstimator(startPose);

        currentDistanceCM = distanceToGoalCM(startPose);

        telemetry.addLine("Blue Side Close Comp Initialised");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        updateGoalTarget();

        Pose currentPose = follower.getPose();
        PoseStorage.currentPose = currentPose;

        currentDistanceCM = distanceToGoalCM(currentPose);

        updateShooterRegressionAndPIDF(currentPose);

        telemetry.addLine("Blue Side Close Comp Init Loop");

        telemetry.addLine("Goal / Distance");
        telemetry.addData("Goal X", "%.2f", GOAL_X);
        telemetry.addData("Goal Y", "%.2f", GOAL_Y);
        telemetry.addData("Distance CM", "%.1f", currentDistanceCM);

        telemetry.addLine("Shooter");
        telemetry.addData("Target TPS", "%.0f", Outtake.target);
        telemetry.addData("Current TPS", "%.0f", Outtake.currentTPS);

        telemetry.addLine("Hood");
        telemetry.addData("Hood Command", "%.3f", RobotHardware.lastCommandedHood);

        telemetry.addLine("Turret");
        telemetry.addData("Aim Offset", "%.4f", Turret.AIM_OFFSET);
        telemetry.addData("Aim Gain", "%.3f", Turret.AIM_GAIN);
        telemetry.addData("Velocity Lead Gain", "%.4f", Turret.VELOCITY_LEAD_GAIN);

        telemetry.update();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Start
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void start() {
        Scheduler.reset();

        robotHardware.reset_all();
        turret.setForward();

        PoseStorage.currentPose = startPose;

        resetVelocityEstimator(startPose);
        updateGoalTarget();

        currentDistanceCM = distanceToGoalCM(startPose);

        outtakeEnabled = true;
        tpsTimeoutFired = false;

        stopIntakeAndBlock();
        updateShooterRegressionAndPIDF(startPose);

        schedule(
                infinite(this::robotPeriodic),

                sequential(
                        // Path 1: drive to shooting position. Intake OFF.
                        followWithTimeout(pathToShoot, TIMEOUT_PATH_1),

                        shootCycle(),

                        // Path 2: drive to Stack 1 with intake ON and blocker CLOSED.
                        followCollectWithTimeout(pathToStack1, TIMEOUT_PATH_2),

                        // Path 3: return to shooting position. Intake OFF.
                        followWithTimeout(pathReturnFromStack1, TIMEOUT_PATH_3),

                        shootCycle(),

                        // Path 4: drive to Stack 2 area.
                        followWithTimeout(pathToStack2, TIMEOUT_PATH_4),

                        // Path 5: collect Stack 2.
                        followCollectWithTimeout(pathToEatStack2, TIMEOUT_PATH_5),

                        // Path 6: return to shooting position.
                        followWithTimeout(pathReturnFromStack2, TIMEOUT_PATH_6),

                        shootCycle(),

                        // Path 7: drive to overflow gate.
                        followWithTimeout(pathToOverflow, TIMEOUT_PATH_7),

                        // Wait at overflow gate while collecting.
                        gateCollectWait(),

                        // Path 8: return to shooting position.
                        followWithTimeout(pathReturnFromOverflow, TIMEOUT_PATH_8),

                        shootCycle(),

                        // Path 9: move to end position.
                        followWithTimeout(pathToEnd, TIMEOUT_PATH_9),

                        instant(this::finishAuto)
                )
        );
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Main loop
    //
    // Scheduler.execute() runs:
    // - robotPeriodic()
    // - active path commands
    // - active shooter/intake commands
    //
    // Telemetry only reads stored values.
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void loop() {
        Scheduler.execute();

        telemetry.addLine("Blue Side Close Comp");

        telemetry.addLine("Auto State");
        telemetry.addData("Outtake Enabled", outtakeEnabled);
        telemetry.addData("TPS Timeout Fired", tpsTimeoutFired);
        telemetry.addData("At Speed", outtake.isAtSpeed(TPS_READY_THRESHOLD));
        telemetry.addData("Distance CM", "%.1f", currentDistanceCM);

        telemetry.addLine("Velocity");
        telemetry.addData("Raw X", "%.2f", rawVelocityX);
        telemetry.addData("Raw Y", "%.2f", rawVelocityY);
        telemetry.addData("Estimated X", "%.2f", estimatedVelocityX);
        telemetry.addData("Estimated Y", "%.2f", estimatedVelocityY);
        telemetry.addData("dt", "%.3f", velocityDt);

        telemetry.addLine("Shooter");
        telemetry.addData("Target TPS", "%.0f", Outtake.target);
        telemetry.addData("Current TPS", "%.0f", Outtake.currentTPS);
        telemetry.addData("Effective TPS", "%.0f", Outtake.effectiveTPS);
        telemetry.addLine("Turret");
        telemetry.addData("Velocity Comp", Turret.velocityCompensationActive);
        telemetry.addData("Aim Offset", "%.4f", Turret.AIM_OFFSET);
        telemetry.addData("Aim Gain", "%.3f", Turret.AIM_GAIN);
        telemetry.addData("Velocity Lead Gain", "%.4f", Turret.VELOCITY_LEAD_GAIN);

        telemetry.update();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Stop
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void stop() {
        PoseStorage.currentPose = follower.getPose();

        outtakeEnabled = false;

        outtake.stopOuttake();
        outtake.updatePIDF();

        stopIntakeAndBlock();
        robotHardware.reset_all();

        turret.setForward();

        Scheduler.reset();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Path building
    // ─────────────────────────────────────────────────────────────────────────

    private void buildPaths() {
        pathToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        pathToStack1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, stack1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), stack1Pose.getHeading())
                .build();

        pathReturnFromStack1 = follower.pathBuilder()
                .addPath(new BezierLine(stack1Pose, shootPose))
                .setLinearHeadingInterpolation(stack1Pose.getHeading(), shootPose.getHeading())
                .build();

        pathToStack2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, stack2Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), stack2Pose.getHeading())
                .build();

        pathToEatStack2 = follower.pathBuilder()
                .addPath(new BezierLine(stack2Pose, eatStack2Pose))
                .setLinearHeadingInterpolation(stack2Pose.getHeading(), eatStack2Pose.getHeading())
                .build();

        pathReturnFromStack2 = follower.pathBuilder()
                .addPath(new BezierLine(eatStack2Pose, shootPose))
                .setLinearHeadingInterpolation(eatStack2Pose.getHeading(), shootPose.getHeading())
                .build();

        pathToOverflow = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, overflowPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), overflowPose.getHeading())
                .build();

        pathReturnFromOverflow = follower.pathBuilder()
                .addPath(new BezierLine(overflowPose, shootPose))
                .setLinearHeadingInterpolation(overflowPose.getHeading(), shootPose.getHeading())
                .build();

        pathToEnd = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
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
                instant(() -> {
                    updateShooterRegressionAndPIDF(PoseStorage.currentPose);
                    robotHardware.block();
                    intake.intake(COLLECT_INTAKE_LEFT_POWER, COLLECT_INTAKE_RIGHT_POWER);
                }),

                waitSeconds(GATE_COLLECT_SECONDS),

                instant(() -> {
                    updateShooterRegressionAndPIDF(PoseStorage.currentPose);
                    intake.intakeStop();
                    robotHardware.block();
                })
        );
    }

    private Command shootCycle() {
        return sequential(
                instant(() -> {
                    startTpsWait();

                    outtakeEnabled = true;
                    updateShooterRegressionAndPIDF(PoseStorage.currentPose);

                    stopIntakeAndBlock();
                }),

                race(
                        waitUntil(this::shooterReadyOrTimedOut),
                        infinite(this::holdShooterAndBlock)
                ),

                race(
                        waitSeconds(SHOOT_SETTLE_SECONDS),
                        infinite(this::holdShooterAndBlock)
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
    // Timing-critical order:
    // 1. follower.update()
    // 2. read currentPose once
    // 3. update pose storage
    // 4. update velocity from same pose
    // 5. calculate distance from same pose
    // 6. aim turret from same pose
    // 7. update shooter and hood from same distance
    // ─────────────────────────────────────────────────────────────────────────

    private void robotPeriodic() {
        follower.update();

        Pose currentPose = follower.getPose();
        PoseStorage.currentPose = currentPose;

        updateEstimatedVelocity(currentPose);

        updateGoalTarget();

        currentDistanceCM = distanceToGoalCM(currentPose);

        turret.aimTurret(
                currentPose,
                estimatedVelocityX,
                estimatedVelocityY
        );

        if (outtakeEnabled) {
            updateShooterRegressionAndPIDF(currentPose);
        } else {
            outtake.stopOuttake();
            outtake.updatePIDF();
        }
    }

    private void finishAuto() {
        outtakeEnabled = false;

        outtake.stopOuttake();
        outtake.updatePIDF();

        stopIntakeAndBlock();
        robotHardware.reset_all();

        turret.setForward();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Shooter update helper
    //
    // Uses the pose already sampled by robotPeriodic().
    // Does not call follower.getPose().
    // ─────────────────────────────────────────────────────────────────────────

    private void updateShooterRegressionAndPIDF(Pose robotPose) {
        currentDistanceCM = distanceToGoalCM(robotPose);

        outtake.linearRegression(currentDistanceCM);
        robotHardware.linearHoodRegression(currentDistanceCM);

        outtake.updatePIDF();
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
        updateShooterRegressionAndPIDF(PoseStorage.currentPose);

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
    //
    // Collection:
    // - blocker closed
    // - intake on
    //
    // Shooting:
    // - blocker released
    // - intake on
    // ─────────────────────────────────────────────────────────────────────────

    private void stopIntakeAndBlock() {
        intake.intakeStop();
        robotHardware.block();
    }

    private void holdShooterAndBlock() {
        updateShooterRegressionAndPIDF(PoseStorage.currentPose);

        intake.intakeStop();
        robotHardware.block();
    }

    private void runShootFeed() {
        updateShooterRegressionAndPIDF(PoseStorage.currentPose);

        robotHardware.release();
        intake.intake(SHOOT_INTAKE_LEFT_POWER, SHOOT_INTAKE_RIGHT_POWER);
    }

    private void runGateCollect() {
        updateShooterRegressionAndPIDF(PoseStorage.currentPose);

        robotHardware.block();
        intake.intake(COLLECT_INTAKE_LEFT_POWER, COLLECT_INTAKE_RIGHT_POWER);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Velocity estimation
    // ─────────────────────────────────────────────────────────────────────────

    private void resetVelocityEstimator(Pose pose) {
        previousVelocityPose = pose;
        previousVelocityTime = getRuntime();

        rawVelocityX = 0.0;
        rawVelocityY = 0.0;

        estimatedVelocityX = 0.0;
        estimatedVelocityY = 0.0;

        velocityDt = 0.0;
    }

    private void updateEstimatedVelocity(Pose currentPose) {
        if (!USE_POSE_DELTA_VELOCITY || currentPose == null) {
            rawVelocityX = 0.0;
            rawVelocityY = 0.0;

            estimatedVelocityX = 0.0;
            estimatedVelocityY = 0.0;

            velocityDt = 0.0;
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

        double alpha = clamp(VELOCITY_SMOOTHING_ALPHA);

        estimatedVelocityX =
                (alpha * rawVelocityX) +
                        ((1.0 - alpha) * estimatedVelocityX);

        estimatedVelocityY =
                (alpha * rawVelocityY) +
                        ((1.0 - alpha) * estimatedVelocityY);

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

    private double distanceToGoalCM(Pose robotPose) {
        if (robotPose == null) {
            return currentDistanceCM;
        }

        Pose goalPose = getGoalPose();

        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();

        return Math.hypot(dx, dy) * 2.54;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Utility
    // ─────────────────────────────────────────────────────────────────────────

    private double clamp(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }
}