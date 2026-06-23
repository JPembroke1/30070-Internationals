package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.opModes.subClasses.Intake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Outtake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.RobotHardware;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Configurable
@Autonomous(name = "00 Gate Auto 15 Blue", group = "Blue")
public class GateAuto15Blue extends OpMode {

    private Follower follower;
    private Intake intake;
    private Outtake outtake;
    private Turret turret;
    private RobotHardware robotHardware;

    private List<LynxModule> allHubs;

    public static double GOAL_X = 6.0;
    public static double GOAL_Y = 138.0;

    public static double SHOOT_SETTLE_SECONDS = 0.3;
    public static double SHOOT_FEED_SECONDS = 0.6;
    public static double GATE_COLLECT_SECONDS = 2.5;

    public static double RETURN_INTAKE_SECONDS = 0.35;

    public static double SHOOT_INTAKE_LEFT_POWER = 0.6;
    public static double SHOOT_INTAKE_RIGHT_POWER = 0.6;

    public static double COLLECT_INTAKE_LEFT_POWER = 1.0;
    public static double COLLECT_INTAKE_RIGHT_POWER = 1.0;

    public static double SHOOT_PATH_POWER = 1.0;
    public static double STACK_PATH_POWER = 0.80;
    public static double STACK_EXIT_PATH_POWER = 0.75;
    public static double GATE_PATH_POWER = 0.65;
    public static double END_PATH_POWER = 1.0;

    public static double GATE_FOLLOW_TIMEOUT_SECONDS = 1.0;

    public static double TPS_SPINUP_TIMEOUT = 0.3;
    public static double TPS_READY_THRESHOLD = 0.95;

    private double tpsWaitStartTime = 0.0;
    private boolean tpsTimeoutFired = false;
    private boolean outtakeEnabled = true;

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

    private final Pose startPose = new Pose(21, 123, Math.toRadians(145));
    private final Pose shootPose = new Pose(60, 86, Math.toRadians(180));

    private final Pose stack1Pose = new Pose(17, 84, Math.toRadians(180));

    private final Pose stack2Pose = new Pose(40, 62, Math.toRadians(180));
    private final Pose eatStack2Pose = new Pose(15, 62, Math.toRadians(180));

    private final Pose overflowPose = new Pose(15, 62, Math.toRadians(158));
    private final Pose endPose = new Pose(59, 101, Math.toRadians(145));
    private final Pose toOverflowPose = new Pose(45, 62, Math.toRadians(180));

    private PathChain pathToShoot;
    private PathChain pathToStack2;
    private PathChain pathToCollectStack2;
    private PathChain pathToExitStack2;
    private PathChain pathFromStack2ToShoot;
    private PathChain pathToOffsetOverflow;
    private PathChain pathToGate;
    private PathChain pathToShoot3;
    private PathChain pathToGate2;
    private PathChain pathToShoot4;
    private PathChain pathToStack1;
    private PathChain pathToEnd;

    @Override
    public void init() {
        Scheduler.reset();

        configureBulkCaching();
        clearBulkCache();

        PoseStorage.setBlue();
        PoseStorage.setPose(startPose);

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

        telemetry.addLine("Gate Blue Initialised");
        telemetry.addData("Alliance Stored", PoseStorage.lastAlliance);
        telemetry.addData("Bulk Caching", "MANUAL");
        telemetry.addData("Path Completion", "Pedro follow completion");
        telemetry.addData("Stack 2 Return", "Exit stack first, then return to shoot");
        telemetry.addData("Gate Wait", "Time OR intake.isBallReady()");
        telemetry.addData(
                "Overflow Pose",
                "(%.1f, %.1f, %.1f deg)",
                overflowPose.getX(),
                overflowPose.getY(),
                Math.toDegrees(overflowPose.getHeading())
        );
        telemetry.addData("Gate Follow Timeout", "%.2f seconds", GATE_FOLLOW_TIMEOUT_SECONDS);
        telemetry.addData("Return Intake", "%.2f seconds", RETURN_INTAKE_SECONDS);
        telemetry.addData("Stack Exit Power", "%.2f", STACK_EXIT_PATH_POWER);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        clearBulkCache();

        PoseStorage.setBlue();
        updateGoalTarget();

        Pose currentPose = follower.getPose();
        PoseStorage.setPose(currentPose);

        currentDistanceCM = distanceToGoalCM(currentPose);

        intake.update();

        telemetry.addLine("Gate Blue Init Loop");
        telemetry.addData("Alliance Stored", PoseStorage.lastAlliance);
        telemetry.addData("Distance CM", "%.1f", currentDistanceCM);
        telemetry.addData("Target TPS", "%.0f", Outtake.target);
        telemetry.addData("Current TPS", "%.0f", Outtake.currentTPS);
        telemetry.update();
    }

    @Override
    public void start() {
        Scheduler.reset();

        clearBulkCache();

        PoseStorage.setBlue();
        PoseStorage.setPose(startPose);

        robotHardware.reset_all();
        turret.setForward();

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
                        setFollowerPower(SHOOT_PATH_POWER),
                        followPath(pathToShoot),

                        shootCycle(),

                        setFollowerPower(STACK_PATH_POWER),
                        followCollect(pathToStack2),

                        followCollect(pathToCollectStack2),

                        setFollowerPower(STACK_EXIT_PATH_POWER),
                        followReturnWithTimedIntake(pathToExitStack2),

                        setFollowerPower(SHOOT_PATH_POWER),
                        followReturnWithTimedIntake(pathFromStack2ToShoot),

                        shootCycle(),

                        followPath(pathToOffsetOverflow),

                        setFollowerPower(GATE_PATH_POWER),
                        followCollectWithTimeout(pathToGate, GATE_FOLLOW_TIMEOUT_SECONDS),

                        gateCollectWait(),

                        setFollowerPower(SHOOT_PATH_POWER),
                        followReturnWithTimedIntake(pathToShoot3),

                        shootCycle(),

                        followPath(pathToOffsetOverflow),

                        setFollowerPower(GATE_PATH_POWER),
                        followCollectWithTimeout(pathToGate2, GATE_FOLLOW_TIMEOUT_SECONDS),

                        gateCollectWait(),

                        setFollowerPower(SHOOT_PATH_POWER),
                        followReturnWithTimedIntake(pathToShoot4),

                        shootCycle(),

                        setFollowerPower(STACK_PATH_POWER),
                        followCollect(pathToStack1),

                        setFollowerPower(END_PATH_POWER),
                        followReturnWithTimedIntake(pathToEnd),

                        shootCycle(),

                        instant(this::finishAuto)
                )
        );
    }

    @Override
    public void loop() {
        clearBulkCache();

        Scheduler.execute();

        Pose currentPose = follower.getPose();

        telemetry.addLine("Gate Blue Auto");

        telemetry.addLine("State");
        telemetry.addData("Alliance Stored", PoseStorage.lastAlliance);
        telemetry.addData("Outtake Enabled", outtakeEnabled);
        telemetry.addData("TPS Timeout Fired", tpsTimeoutFired);
        telemetry.addData("At Speed", outtake.isAtSpeed(TPS_READY_THRESHOLD));
        telemetry.addData("Distance CM", "%.1f", currentDistanceCM);

        telemetry.addLine("Pose");
        telemetry.addData("X", "%.2f", currentPose.getX());
        telemetry.addData("Y", "%.2f", currentPose.getY());
        telemetry.addData("Heading Deg", "%.1f", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("Overflow X", "%.2f", overflowPose.getX());
        telemetry.addData("Overflow Y", "%.2f", overflowPose.getY());
        telemetry.addData("Overflow H Deg", "%.1f", Math.toDegrees(overflowPose.getHeading()));


        telemetry.addLine("Shooter");
        telemetry.addData("Target TPS", "%.0f", Outtake.target);
        telemetry.addData("Current TPS", "%.0f", Outtake.currentTPS);
        telemetry.addData("Effective TPS", "%.0f", Outtake.effectiveTPS);

        telemetry.addLine("Velocity");
        telemetry.addData("Raw X", "%.2f", rawVelocityX);
        telemetry.addData("Raw Y", "%.2f", rawVelocityY);
        telemetry.addData("Estimated X", "%.2f", estimatedVelocityX);
        telemetry.addData("Estimated Y", "%.2f", estimatedVelocityY);
        telemetry.addData("dt", "%.3f", velocityDt);

        telemetry.addLine("Turret");
        telemetry.addData("Velocity Comp", Turret.velocityCompensationActive);
        telemetry.addData("Aim Offset", "%.4f", Turret.AIM_OFFSET);
        telemetry.addData("Aim Gain", "%.3f", Turret.AIM_GAIN);
        telemetry.addData("Velocity Lead Gain", "%.4f", Turret.VELOCITY_LEAD_GAIN);

        telemetry.update();
    }

    @Override
    public void stop() {
        clearBulkCache();

        PoseStorage.setBlue();
        PoseStorage.setPose(follower.getPose());

        outtakeEnabled = false;

        outtake.stopOuttake();
        outtake.updatePIDF();

        stopIntakeAndBlock();
        robotHardware.reset_all();

        turret.setForward();

        Scheduler.reset();
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

    private void buildPaths() {
        pathToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(
                        startPose.getHeading(),
                        shootPose.getHeading()
                )
                .build();

        pathToStack2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, stack2Pose))
                .setHeadingInterpolation(halfTangentHalfConstant(stack2Pose.getHeading()))
                .build();

        pathToCollectStack2 = follower.pathBuilder()
                .addPath(new BezierLine(stack2Pose, eatStack2Pose))
                .setLinearHeadingInterpolation(
                        stack2Pose.getHeading(),
                        eatStack2Pose.getHeading()
                )
                .build();

        pathToExitStack2 = follower.pathBuilder()
                .addPath(new BezierLine(eatStack2Pose, stack2Pose))
                .setLinearHeadingInterpolation(
                        eatStack2Pose.getHeading(),
                        stack2Pose.getHeading()
                )
                .build();

        pathFromStack2ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(stack2Pose, shootPose))
                .setLinearHeadingInterpolation(
                        stack2Pose.getHeading(),
                        shootPose.getHeading()
                )
                .build();

        pathToGate = follower.pathBuilder()
                .addPath(new BezierLine(endPose, overflowPose))
                .setLinearHeadingInterpolation(
                        shootPose.getHeading(),
                        overflowPose.getHeading()
                )
                .build();

        pathToShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(overflowPose, shootPose))
                .setLinearHeadingInterpolation(
                        overflowPose.getHeading(),
                        shootPose.getHeading()
                )
                .build();

        pathToGate2 = follower.pathBuilder()
                .addPath(new BezierLine(endPose, overflowPose))
                .setLinearHeadingInterpolation(
                        shootPose.getHeading(),
                        overflowPose.getHeading()
                )
                .build();

        pathToShoot4 = follower.pathBuilder()
                .addPath(new BezierLine(overflowPose, shootPose))
                .setLinearHeadingInterpolation(
                        overflowPose.getHeading(),
                        shootPose.getHeading()
                )
                .build();

        pathToStack1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, stack1Pose))
                .setHeadingInterpolation(halfTangentHalfConstant(stack1Pose.getHeading()))
                .build();

        pathToEnd = follower.pathBuilder()
                .addPath(new BezierLine(stack1Pose, endPose))
                .setLinearHeadingInterpolation(
                        stack1Pose.getHeading(),
                        endPose.getHeading()
                )
                .build();

        pathToOffsetOverflow = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, toOverflowPose))
                .setLinearHeadingInterpolation(
                        shootPose.getHeading(),
                        toOverflowPose.getHeading()
                )
                .build();
    }

    private HeadingInterpolator halfTangentHalfConstant(double finalHeadingRadians) {
        return HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(
                        0.0,
                        0.5,
                        HeadingInterpolator.tangent
                ),
                new HeadingInterpolator.PiecewiseNode(
                        0.5,
                        1.0,
                        HeadingInterpolator.constant(finalHeadingRadians)
                )
        );
    }

    private Command waitSeconds(double seconds) {
        return waitMs(seconds * 1000.0);
    }

    private Command followPath(PathChain path) {
        return follow(follower, path);
    }

    private Command followPathWithTimeout(PathChain path, double timeoutSeconds) {
        return race(
                follow(follower, path),
                waitSeconds(timeoutSeconds)
        );
    }

    private Command followCollect(PathChain path) {
        return parallel(
                follow(follower, path),

                sequential(
                        instant(() -> {
                            robotHardware.block();
                            intake.intake(
                                    COLLECT_INTAKE_LEFT_POWER,
                                    COLLECT_INTAKE_RIGHT_POWER
                            );
                        })
                )
        );
    }

    private Command followCollectWithTimeout(PathChain path, double timeoutSeconds) {
        return parallel(
                followPathWithTimeout(path, timeoutSeconds),

                sequential(
                        instant(() -> {
                            robotHardware.block();
                            intake.intake(
                                    COLLECT_INTAKE_LEFT_POWER,
                                    COLLECT_INTAKE_RIGHT_POWER
                            );
                        })
                )
        );
    }

    private Command followReturnWithTimedIntake(PathChain path) {
        return parallel(
                follow(follower, path),

                sequential(
                        instant(() -> {
                            robotHardware.block();
                            intake.intake(
                                    COLLECT_INTAKE_LEFT_POWER,
                                    COLLECT_INTAKE_RIGHT_POWER
                            );
                        }),

                        waitSeconds(RETURN_INTAKE_SECONDS),

                        instant(() -> {
                            intake.intakeStop();
                            robotHardware.block();
                        })
                )
        );
    }

    private Command setFollowerPower(double power) {
        return instant(() -> follower.setMaxPower(power));
    }

    private Command gateCollectWait() {
        return sequential(
                instant(() -> {
                    updateShooterRegressionAndPIDF(PoseStorage.currentPose);

                    robotHardware.block();
                    intake.intake(
                            COLLECT_INTAKE_LEFT_POWER,
                            COLLECT_INTAKE_RIGHT_POWER
                    );

                }),

                race(
                        waitSeconds(GATE_COLLECT_SECONDS)
                ),

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

    private void robotPeriodic() {
        follower.update();

        Pose currentPose = follower.getPose();
        PoseStorage.setPose(currentPose);

        updateEstimatedVelocity(currentPose);

        updateGoalTarget();

        currentDistanceCM = distanceToGoalCM(currentPose);

        intake.update();

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
        follower.setMaxPower(SHOOT_PATH_POWER);
    }

    private void updateShooterRegressionAndPIDF(Pose robotPose) {
        currentDistanceCM = distanceToGoalCM(robotPose);

        outtake.linearRegression(currentDistanceCM);
        robotHardware.linearHoodRegression(currentDistanceCM);

        outtake.updatePIDF();
    }

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
        intake.intake(
                SHOOT_INTAKE_LEFT_POWER,
                SHOOT_INTAKE_RIGHT_POWER
        );
    }

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

        double alpha = clamp01(VELOCITY_SMOOTHING_ALPHA);

        estimatedVelocityX =
                (alpha * rawVelocityX) +
                        ((1.0 - alpha) * estimatedVelocityX);

        estimatedVelocityY =
                (alpha * rawVelocityY) +
                        ((1.0 - alpha) * estimatedVelocityY);

        previousVelocityPose = currentPose;
        previousVelocityTime = currentTime;
    }

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

    private double clamp01(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }
}