package org.firstinspires.ftc.teamcode.opModes;

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
    // Goal position
    // Tuning note: adjust GOAL_OFFSET_X/Y if turret aim is consistently off.
    // ─────────────────────────────────────────────────────────────────────────

    public static double GOAL_X = 6;
    public static double GOAL_Y = 138;

    public static double GOAL_OFFSET_X = 0;
    public static double GOAL_OFFSET_Y = 0;

    // ─────────────────────────────────────────────────────────────────────────
    // Shooting and collection timing
    // Tuning notes:
    // - If first shot is weak, increase SHOOT_SETTLE_SECONDS.
    // - If not all balls feed, increase SHOOT_FEED_SECONDS.
    // - If gate collection is unreliable, increase GATE_COLLECT_SECONDS.
    // ─────────────────────────────────────────────────────────────────────────

    public static double SHOOT_SETTLE_SECONDS = 0.35;
    public static double SHOOT_FEED_SECONDS = 2.0;
    public static double GATE_COLLECT_SECONDS = 2.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Intake powers
    // Tuning note: shooting feed power should usually be gentler than collection.
    // ─────────────────────────────────────────────────────────────────────────

    public static double SHOOT_INTAKE_LEFT_POWER = 1.0;
    public static double SHOOT_INTAKE_RIGHT_POWER = 1.0;

    public static double COLLECT_INTAKE_LEFT_POWER = 1.0;
    public static double COLLECT_INTAKE_RIGHT_POWER = 1.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Path timeouts
    // Tuning note: reduce if auto waits too long after paths; increase if paths
    // are being cut short before the robot reaches position.
    // ─────────────────────────────────────────────────────────────────────────

    private static final double TIMEOUT_PATH_1 = 4.0;
    private static final double TIMEOUT_PATH_2 = 3.0;
    private static final double TIMEOUT_PATH_3 = 3.0;
    private static final double TIMEOUT_PATH_4 = 4.0;
    private static final double TIMEOUT_PATH_5 = 2.0;
    private static final double TIMEOUT_PATH_6 = 4.0;
    private static final double TIMEOUT_PATH_7 = 3.0;
    private static final double TIMEOUT_PATH_8 = 3.0;
    private static final double TIMEOUT_PATH_9 = 3.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Shooter readiness
    // Tuning note: 0.95 means shooter is ready at 95% of target TPS.
    // Example: 1500 TPS target × 0.95 = 1425 TPS ready threshold.
    // ─────────────────────────────────────────────────────────────────────────

    private static final double TPS_SPINUP_TIMEOUT = 2.0;
    private static final double TPS_READY_THRESHOLD = 0.95;

    private double tpsWaitStartTime = 0;
    private boolean tpsTimeoutFired = false;
    private boolean outtakeEnabled = true;

    // ─────────────────────────────────────────────────────────────────────────
    // Field poses
    // ─────────────────────────────────────────────────────────────────────────

    private final Pose startPose  = new Pose(21, 121, Math.toRadians(143));
    private final Pose Shoot      = new Pose(60, 80,  Math.toRadians(177));
    private final Pose Stack_1    = new Pose(17, 82,  Math.toRadians(177));
    private final Pose Stack_2    = new Pose(40, 56,  Math.toRadians(170));
    private final Pose EatStack_2 = new Pose(15, 60,  Math.toRadians(170));
    private final Pose OverFlow   = new Pose(13, 60,  Math.toRadians(150));
    private final Pose End        = new Pose(50, 70,  Math.toRadians(0));

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
        turret.init(hardwareMap);

        updateGoalTarget();
    }

    @Override
    public void init_loop() {
        updateGoalTarget();

        double distCM = distanceToGoalCM();
        outtake.linearRegression(distCM);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Start command schedule
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void start() {
        Scheduler.reset();

        robotHardware.reset_all();
        turret.centre();

        PoseStorage.currentPose = startPose;
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
    }

    @Override
    public void stop() {
        PoseStorage.currentPose = follower.getPose();

        outtakeEnabled = false;
        outtake.stopOuttake();
        outtake.updatePIDF();

        stopIntakeAndBlock();
        robotHardware.reset_all();

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
    // Keeps Pedro, pose, turret, hood and shooter active during all commands.
    // ─────────────────────────────────────────────────────────────────────────

    private void robotPeriodic() {
        follower.update();
        updateGoalTarget();

        Pose currentPose = follower.getPose();
        PoseStorage.currentPose = currentPose;

        turret.aimTurret(currentPose);

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
        stopIntakeAndBlock();
        robotHardware.reset_all();
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
        if (isShooterAtSpeed()) return true;

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
    // Goal and distance helpers
    // ─────────────────────────────────────────────────────────────────────────

    private Pose getGoalPose() {
        return new Pose(GOAL_X + GOAL_OFFSET_X, GOAL_Y + GOAL_OFFSET_Y, 0);
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
}