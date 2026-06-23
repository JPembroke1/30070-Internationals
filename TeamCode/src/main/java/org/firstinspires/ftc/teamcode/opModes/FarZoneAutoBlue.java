package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
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
@Autonomous(name = "00 Far Zone Auto Blue", group = "Blue")
public class FarZoneAutoBlue extends OpMode {

    private Follower follower;
    private Intake intake;
    private Outtake outtake;
    private Turret turret;
    private RobotHardware robotHardware;

    private List<LynxModule> allHubs;

    public static double GOAL_X = 6.0;
    public static double GOAL_Y = 138.0;

    public static double FAR_ZONE_TARGET_TPS = 3400.0;
    public static double FAR_ZONE_HOOD_DISTANCE_CM = 200.0;

    public static double SHOOT_SETTLE_SECONDS = 0.3;
    public static double SHOOT_FEED_SECONDS = 0.4;

    public static double TIMED_INTAKE_SECONDS = 0.35;

    public static double SHOOT_INTAKE_LEFT_POWER = 1.0;
    public static double SHOOT_INTAKE_RIGHT_POWER = 1.0;

    public static double COLLECT_INTAKE_LEFT_POWER = 1.0;
    public static double COLLECT_INTAKE_RIGHT_POWER = 1.0;

    public static double SHOOT_PATH_POWER = 1.0;
    public static double WALL_STACK_PATH_POWER = 0.8;
    public static double STACK_3_PATH_POWER = 0.75;
    public static double OVERFLOW_PATH_POWER = 0.65;
    public static double END_PATH_POWER = 1.0;

    /*
     * IMPORTANT:
     * The old value of 0.3 seconds was allowing the auto to continue
     * before the shooter had actually reached target speed.
     */
    public static boolean REQUIRE_TPS_BEFORE_SHOOT = true;
    public static double TPS_SPINUP_TIMEOUT = 2.0;
    public static double TPS_READY_THRESHOLD = 0.97;
    public static double TPS_READY_STABLE_SECONDS = 0.15;

    private double tpsWaitStartTime = 0.0;
    private double shooterReadyStartTime = -1.0;

    private boolean tpsTimeoutFired = false;
    private boolean shooterStableReady = false;
    private boolean outtakeEnabled = true;

    private double currentDistanceCM = 0.0;

    private final Pose shootPose = new Pose(56, 8.5, Math.toRadians(180));
    private final Pose wallStackPose = new Pose(12, 8.5, Math.toRadians(180));
    private final Pose shoot2Pose = new Pose(56, 8.5, Math.toRadians(180));

    private final Pose stack3Pose = new Pose(45, 35, Math.toRadians(180));
    private final Pose stack3CollectPose = new Pose(13, 35, Math.toRadians(180));
    private final Pose shoot3Pose = new Pose(56, 8.5, Math.toRadians(180));

    private final Pose overflowPose = new Pose(12, 18, Math.toRadians(180));
    private final Pose shoot4Pose = new Pose(56, 8.5, Math.toRadians(180));

    private final Pose endPose = new Pose(38, 32, Math.toRadians(90));

    private PathChain pathToWallStack;
    private PathChain pathFromWallStackToShoot2;
    private PathChain pathToStack3;
    private PathChain pathToStack3Collect;
    private PathChain pathFromStack3CollectToShoot3;
    private PathChain pathToOverflow;
    private PathChain pathFromOverflowToShoot4;
    private PathChain pathToEnd;

    @Override
    public void init() {
        Scheduler.reset();

        configureBulkCaching();
        clearBulkCache();

        PoseStorage.setBlue();
        PoseStorage.setPose(shootPose);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(shootPose);

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

        currentDistanceCM = distanceToGoalCM(shootPose);

        telemetry.addLine("Far Zone Auto Blue Initialised");
        telemetry.addData("Alliance Stored", PoseStorage.lastAlliance);
        telemetry.addData("Bulk Caching", "MANUAL");
        telemetry.addData(
                "Start/Shoot Pose",
                "(%.1f, %.1f, %.1f deg)",
                shootPose.getX(),
                shootPose.getY(),
                Math.toDegrees(shootPose.getHeading())
        );
        telemetry.addData("Far TPS", "%.0f", FAR_ZONE_TARGET_TPS);
        telemetry.addData("Hood Distance CM", "%.0f", FAR_ZONE_HOOD_DISTANCE_CM);
        telemetry.addData("TPS Required", REQUIRE_TPS_BEFORE_SHOOT);
        telemetry.addData("TPS Ready Threshold", "%.2f", TPS_READY_THRESHOLD);
        telemetry.addData("TPS Stable Seconds", "%.2f", TPS_READY_STABLE_SECONDS);
        telemetry.addData("TPS Timeout", "%.2f", TPS_SPINUP_TIMEOUT);
        telemetry.addData("Timed Intake", "%.2f seconds", TIMED_INTAKE_SECONDS);
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

        telemetry.addLine("Far Zone Auto Blue Init Loop");
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
        PoseStorage.setPose(shootPose);

        robotHardware.reset_all();
        turret.setForward();

        updateGoalTarget();

        currentDistanceCM = distanceToGoalCM(shootPose);

        outtakeEnabled = true;
        tpsTimeoutFired = false;
        shooterStableReady = false;
        shooterReadyStartTime = -1.0;

        stopIntakeAndBlock();
        updateFarZoneShooterAndPIDF(shootPose);

        schedule(
                infinite(this::robotPeriodic),

                sequential(
                        shootCycle(),

                        setFollowerPower(WALL_STACK_PATH_POWER),
                        followTimedIntake(pathToWallStack),

                        setFollowerPower(SHOOT_PATH_POWER),
                        followTimedIntake(pathFromWallStackToShoot2),

                        shootCycle(),

                        setFollowerPower(STACK_3_PATH_POWER),
                        followCollect(pathToStack3),

                        followTimedIntake(pathToStack3Collect),

                        setFollowerPower(SHOOT_PATH_POWER),
                        followTimedIntake(pathFromStack3CollectToShoot3),

                        shootCycle(),

                        setFollowerPower(OVERFLOW_PATH_POWER),
                        followCollect(pathToOverflow),

                        setFollowerPower(SHOOT_PATH_POWER),
                        followTimedIntake(pathFromOverflowToShoot4),

                        shootCycle(),

                        setFollowerPower(END_PATH_POWER),
                        followPath(pathToEnd),

                        instant(this::finishAuto)
                )
        );
    }

    @Override
    public void loop() {
        clearBulkCache();

        Scheduler.execute();

        Pose currentPose = follower.getPose();

        telemetry.addLine("===== FAR ZONE AUTO BLUE =====");

        telemetry.addLine("State");
        telemetry.addData("Alliance Stored", PoseStorage.lastAlliance);
        telemetry.addData("Outtake Enabled", outtakeEnabled);
        telemetry.addData("Require TPS", REQUIRE_TPS_BEFORE_SHOOT);
        telemetry.addData("TPS Timeout Fired", tpsTimeoutFired);
        telemetry.addData("At Speed", outtake.isAtSpeed(TPS_READY_THRESHOLD));
        telemetry.addData("Stable Ready", shooterStableReady);
        telemetry.addData("Distance CM", "%.1f", currentDistanceCM);

        telemetry.addLine("Pose");
        telemetry.addData("X", "%.2f", currentPose.getX());
        telemetry.addData("Y", "%.2f", currentPose.getY());
        telemetry.addData("Heading Deg", "%.1f", Math.toDegrees(currentPose.getHeading()));

        telemetry.addLine("Shooter");
        telemetry.addData("Far Target TPS", "%.0f", FAR_ZONE_TARGET_TPS);
        telemetry.addData("Target TPS", "%.0f", Outtake.target);
        telemetry.addData("Current TPS", "%.0f", Outtake.currentTPS);
        telemetry.addData("Effective TPS", "%.0f", Outtake.effectiveTPS);
        telemetry.addData("Ready Threshold", "%.2f", TPS_READY_THRESHOLD);
        telemetry.addData("Stable Seconds", "%.2f", TPS_READY_STABLE_SECONDS);

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
        pathToWallStack = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, wallStackPose))
                .setLinearHeadingInterpolation(
                        shootPose.getHeading(),
                        wallStackPose.getHeading()
                )
                .build();

        pathFromWallStackToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(wallStackPose, shoot2Pose))
                .setLinearHeadingInterpolation(
                        wallStackPose.getHeading(),
                        shoot2Pose.getHeading()
                )
                .build();

        pathToStack3 = follower.pathBuilder()
                .addPath(new BezierLine(shoot2Pose, stack3Pose))
                .setLinearHeadingInterpolation(
                        shoot2Pose.getHeading(),
                        stack3Pose.getHeading()
                )
                .build();

        pathToStack3Collect = follower.pathBuilder()
                .addPath(new BezierLine(stack3Pose, stack3CollectPose))
                .setLinearHeadingInterpolation(
                        stack3Pose.getHeading(),
                        stack3CollectPose.getHeading()
                )
                .build();

        pathFromStack3CollectToShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(stack3CollectPose, shoot3Pose))
                .setLinearHeadingInterpolation(
                        stack3CollectPose.getHeading(),
                        shoot3Pose.getHeading()
                )
                .build();

        pathToOverflow = follower.pathBuilder()
                .addPath(new BezierLine(shoot3Pose, overflowPose))
                .setLinearHeadingInterpolation(
                        shoot3Pose.getHeading(),
                        overflowPose.getHeading()
                )
                .build();

        pathFromOverflowToShoot4 = follower.pathBuilder()
                .addPath(new BezierLine(overflowPose, shoot4Pose))
                .setLinearHeadingInterpolation(
                        overflowPose.getHeading(),
                        shoot4Pose.getHeading()
                )
                .build();

        pathToEnd = follower.pathBuilder()
                .addPath(new BezierLine(shoot4Pose, endPose))
                .setLinearHeadingInterpolation(
                        shoot4Pose.getHeading(),
                        endPose.getHeading()
                )
                .build();
    }

    private Command waitSeconds(double seconds) {
        return waitMs(seconds * 1000.0);
    }

    private Command followPath(PathChain path) {
        return follow(follower, path);
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

    private Command followTimedIntake(PathChain path) {
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

                        waitSeconds(TIMED_INTAKE_SECONDS),

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

    private Command shootCycle() {
        return sequential(
                instant(() -> {
                    startTpsWait();

                    outtakeEnabled = true;
                    shooterStableReady = false;

                    updateFarZoneShooterAndPIDF(PoseStorage.currentPose);

                    stopIntakeAndBlock();
                }),

                race(
                        waitUntil(this::shooterStableReadyOrAllowedToContinue),
                        infinite(this::holdShooterAndBlock)
                ),

                race(
                        waitSeconds(SHOOT_SETTLE_SECONDS),
                        infinite(this::holdShooterAndBlock)
                ),

                race(
                        waitSeconds(SHOOT_FEED_SECONDS),
                        infinite(this::runShootFeedOnlyWhenReady)
                ),

                instant(this::stopIntakeAndBlock)
        );
    }

    private void robotPeriodic() {
        follower.update();

        Pose currentPose = follower.getPose();
        PoseStorage.setPose(currentPose);

        updateGoalTarget();

        currentDistanceCM = distanceToGoalCM(currentPose);

        intake.update();

        turret.aimTurret(
                currentPose,
                0.0,
                0.0
        );

        if (outtakeEnabled) {
            updateFarZoneShooterAndPIDF(currentPose);
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

    private void updateFarZoneShooterAndPIDF(Pose robotPose) {
        currentDistanceCM = distanceToGoalCM(robotPose);

        outtake.setFarZoneTargetTPS(FAR_ZONE_TARGET_TPS);
        robotHardware.linearHoodRegression(FAR_ZONE_HOOD_DISTANCE_CM);

        outtake.updatePIDF();
    }

    private void startTpsWait() {
        tpsWaitStartTime = getRuntime();
        shooterReadyStartTime = -1.0;

        tpsTimeoutFired = false;
        shooterStableReady = false;
    }

    private boolean hasSpinUpTimedOut() {
        return (getRuntime() - tpsWaitStartTime) > TPS_SPINUP_TIMEOUT;
    }

    private boolean isShooterAtSpeed() {
        return outtake.isAtSpeed(TPS_READY_THRESHOLD);
    }

    private boolean isShooterStableAtSpeed() {
        updateFarZoneShooterAndPIDF(PoseStorage.currentPose);

        if (!isShooterAtSpeed()) {
            shooterReadyStartTime = -1.0;
            shooterStableReady = false;
            return false;
        }

        if (shooterReadyStartTime < 0.0) {
            shooterReadyStartTime = getRuntime();
            shooterStableReady = false;
            return false;
        }

        shooterStableReady = (getRuntime() - shooterReadyStartTime) >= TPS_READY_STABLE_SECONDS;
        return shooterStableReady;
    }

    private boolean shooterStableReadyOrAllowedToContinue() {
        if (isShooterStableAtSpeed()) {
            return true;
        }

        if (hasSpinUpTimedOut()) {
            tpsTimeoutFired = true;

            if (!REQUIRE_TPS_BEFORE_SHOOT) {
                return true;
            }
        }

        return false;
    }

    private void stopIntakeAndBlock() {
        intake.intakeStop();
        robotHardware.block();
    }

    private void holdShooterAndBlock() {
        updateFarZoneShooterAndPIDF(PoseStorage.currentPose);

        intake.intakeStop();
        robotHardware.block();
    }

    private void runShootFeedOnlyWhenReady() {
        updateFarZoneShooterAndPIDF(PoseStorage.currentPose);

        if (!isShooterAtSpeed()) {
            intake.intakeStop();
            robotHardware.block();
            return;
        }

        robotHardware.release();

        intake.intake(
                SHOOT_INTAKE_LEFT_POWER,
                SHOOT_INTAKE_RIGHT_POWER
        );
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
}