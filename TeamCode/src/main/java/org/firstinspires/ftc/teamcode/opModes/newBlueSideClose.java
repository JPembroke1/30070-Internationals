package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opModes.subClasses.Intake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Outtake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.RobotHardware;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Side Close", group = "Blue")
public class newBlueSideClose extends OpMode {

    private Follower follower;
    private Intake intake;
    private Outtake outtake;
    private Turret turret;
    private RobotHardware robotHardware;

    public static double GOAL_X = 0;
    public static double GOAL_Y = 144;

    public static double GOAL_OFFSET_X = 0;
    public static double GOAL_OFFSET_Y = 0;

    // ── Shoot phase timing ───────────────────────────────────────────────────
    // How long the robot holds position and fires at each shoot state
    public static double SHOOT_DURATION_SECONDS = 2.0;

    // ── Path timeouts ────────────────────────────────────────────────────────
    private static final double TIMEOUT_PATH_1 = 4.0;
    private static final double TIMEOUT_PATH_2 = 3.0;
    private static final double TIMEOUT_PATH_3 = 3.0;
    private static final double TIMEOUT_PATH_4 = 4.0;
    private static final double TIMEOUT_PATH_5 = 2.0;
    private static final double TIMEOUT_PATH_6 = 4.0;
    private static final double TIMEOUT_PATH_7 = 3.0;
    private static final double TIMEOUT_PATH_8 = 3.0;
    private static final double TIMEOUT_DONE   = 3.0;

    // ── TPS spin-up ──────────────────────────────────────────────────────────
    private static final double TPS_SPINUP_TIMEOUT  = 2.0;
    private static final double TPS_READY_THRESHOLD = 0.95;

    // ── Timeout tracking ─────────────────────────────────────────────────────
    private double pathStartTime = 0;
    private double currentTimeout = 0;
    private boolean lastTransitionWasTimeout = false;

    private double tpsWaitStartTime = 0;
    private boolean tpsTimeoutFired = false;

    // ── Shoot timer ──────────────────────────────────────────────────────────
    private final ElapsedTime shootTimer = new ElapsedTime();

    // ── State machine ────────────────────────────────────────────────────────
    private enum AutoState {
        PATH_1,
        SHOOT_1,
        PATH_2,
        PATH_3,
        SHOOT_2,
        PATH_4,
        PATH_5,
        PATH_6,
        SHOOT_3,
        PATH_7,
        PATH_8,
        SHOOT_4,
        DONE
    }

    private AutoState state = AutoState.PATH_1;

    // ── Field poses ──────────────────────────────────────────────────────────
    private final Pose startPose  = new Pose(21, 121, Math.toRadians(143));
    private final Pose Shoot      = new Pose(60, 80,  Math.toRadians(177));
    private final Pose Stack_1    = new Pose(17, 82,  Math.toRadians(177));
    private final Pose Stack_2    = new Pose(40, 56,  Math.toRadians(170));
    private final Pose EatStack_2 = new Pose(15, 60,  Math.toRadians(170));
    private final Pose OverFlow   = new Pose(13, 60,  Math.toRadians(150));
    private final Pose End        = new Pose(50, 70,  Math.toRadians(0));

    // ── Paths ────────────────────────────────────────────────────────────────
    private PathChain pathToPos1, pathToPos2, pathToPos3, pathToPos4, pathToPos5;
    private PathChain pathToPos6, pathToPos7, pathToPos8, pathToPos9;

    // ─────────────────────────────────────────────────────────────────────────
    // Path helpers
    // ─────────────────────────────────────────────────────────────────────────

    private void startPath(PathChain path, double timeoutSeconds) {
        follower.followPath(path);
        pathStartTime = getRuntime();
        currentTimeout = timeoutSeconds;
        lastTransitionWasTimeout = false;
    }

    private boolean pathTimedOut() {
        return (getRuntime() - pathStartTime) > currentTimeout;
    }

    private boolean pathComplete() {
        if (!follower.isBusy()) return true;

        if (pathTimedOut()) {
            lastTransitionWasTimeout = true;
            return true;
        }

        return false;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Shooter helpers
    // ─────────────────────────────────────────────────────────────────────────

    private void startTpsWait() {
        tpsWaitStartTime = getRuntime();
        tpsTimeoutFired = false;
    }

    private boolean isShooterAtSpeed() {
        return Outtake.target > 0 && Outtake.currentTPS >= Outtake.target * TPS_READY_THRESHOLD;
    }

    private boolean hasSpinUpTimedOut() {
        return (getRuntime() - tpsWaitStartTime) > TPS_SPINUP_TIMEOUT;
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
    // Shoot state helpers
    // ─────────────────────────────────────────────────────────────────────────

    private void enterShootState() {
        shootTimer.reset();
        startTpsWait();
        stopIntakeAndBlock();
    }

    private boolean shootComplete() {
        return shootTimer.seconds() >= SHOOT_DURATION_SECONDS;
    }

    private void runShootLogic() {
        if (shooterReadyOrTimedOut()) {
            robotHardware.release();
            intake.intake(0.7, 0.9);
        } else {
            robotHardware.block();
            intake.intakeStop();
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Goal / targeting helpers
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

    private double previewTurretServo() {
        Pose robotPose = follower.getPose();
        Pose goalPose = getGoalPose();

        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();

        double targetAngle = Math.atan2(dy, dx);
        double robotHeading = robotPose.getHeading();

        double relativeAngle = targetAngle - robotHeading;
        relativeAngle = Math.atan2(Math.sin(relativeAngle), Math.cos(relativeAngle));

        double preview = 0.5 + (relativeAngle / Math.PI);
        return Math.max(0.0, Math.min(1.0, preview));
    }

    private double previewTurretRelativeAngleDeg() {
        Pose robotPose = follower.getPose();
        Pose goalPose = getGoalPose();

        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();

        double targetAngle = Math.atan2(dy, dx);
        double robotHeading = robotPose.getHeading();

        double relativeAngle = targetAngle - robotHeading;
        relativeAngle = Math.atan2(Math.sin(relativeAngle), Math.cos(relativeAngle));

        return Math.toDegrees(relativeAngle);
    }

    private void stopIntakeAndBlock() {
        intake.intakeStop();
        robotHardware.block();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Build paths
    // ─────────────────────────────────────────────────────────────────────────

    public void buildPaths() {
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

        telemetry.addLine("Initialised. Ready to start.");
        telemetry.addData("Goal X", "%.2f", getGoalPose().getX());
        telemetry.addData("Goal Y", "%.2f", getGoalPose().getY());
        telemetry.update();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Init loop
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void init_loop() {
        updateGoalTarget();

        double distCM = distanceToGoalCM();

        outtake.linearRegression(distCM);

        double previewTurret = previewTurretServo();
        double previewTurretAngleDeg = previewTurretRelativeAngleDeg();

        double previewHood = (RobotHardware.hoodSlope * distCM) + RobotHardware.hoodIntercept;
        previewHood = Math.max(RobotHardware.hoodMin, Math.min(RobotHardware.hoodMax, previewHood));

        telemetry.addLine("─── Ready Check ───");
        telemetry.addData("Distance to Goal (CM)", "%.1f", distCM);
        telemetry.addData("Regression TPS Target", "%.0f", Outtake.target);
        telemetry.addData("Hood Preview", "%.3f", previewHood);
        telemetry.addData("Turret Preview", "%.3f (%.1f°)", previewTurret, previewTurretAngleDeg);
        telemetry.addLine("Servos inactive until START");
        telemetry.update();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Start
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void start() {
        robotHardware.reset_all();
        turret.centre();

        PoseStorage.currentPose = startPose;
        updateGoalTarget();

        startPath(pathToPos1, TIMEOUT_PATH_1);
        startTpsWait();
        state = AutoState.PATH_1;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Loop
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void loop() {
        follower.update();
        updateGoalTarget();

        Pose currentPose = follower.getPose();
        PoseStorage.currentPose = currentPose;

        turret.aimTurret(currentPose);

        double distCM = distanceToGoalCM();
        outtake.linearRegression(distCM);
        robotHardware.linearHoodRegression(distCM);

        outtake.updatePIDF();

        switch (state) {

            case PATH_1:
                // Drive to first shoot position
                // Spin up shooter while driving
                if (shooterReadyOrTimedOut()) {
                    robotHardware.release();
                    intake.intake(1, 1);
                }

                if (pathComplete()) {
                    stopIntakeAndBlock();
                    enterShootState();
                    state = AutoState.SHOOT_1;
                }
                break;

            case SHOOT_1:
                // Stopped at Shoot - fire preload balls
                runShootLogic();

                if (shootComplete()) {
                    stopIntakeAndBlock();
                    startPath(pathToPos2, TIMEOUT_PATH_2);
                    state = AutoState.PATH_2;
                }
                break;

            case PATH_2:
                // Drive to Stack_1
                if (pathComplete()) {
                    stopIntakeAndBlock();
                    startPath(pathToPos3, TIMEOUT_PATH_3);
                    startTpsWait();
                    state = AutoState.PATH_3;
                }
                break;

            case PATH_3:
                // Drive back to Shoot from Stack_1
                // Spin up shooter while returning
                if (shooterReadyOrTimedOut()) {
                    intake.intake(1, 1);
                }

                if (pathComplete()) {
                    stopIntakeAndBlock();
                    enterShootState();
                    state = AutoState.SHOOT_2;
                }
                break;

            case SHOOT_2:
                // Stopped at Shoot - fire Stack_1 balls
                runShootLogic();

                if (shootComplete()) {
                    stopIntakeAndBlock();
                    startPath(pathToPos4, TIMEOUT_PATH_4);
                    state = AutoState.PATH_4;
                }
                break;

            case PATH_4:
                // Drive to Stack_2
                if (pathComplete()) {
                    startPath(pathToPos5, TIMEOUT_PATH_5);
                    startTpsWait();
                    state = AutoState.PATH_5;
                }
                break;

            case PATH_5:
                // Eat Stack_2
                if (shooterReadyOrTimedOut()) {
                    intake.intake(1, 1);
                    robotHardware.release();
                } else {
                    robotHardware.block();
                }

                if (pathComplete()) {
                    stopIntakeAndBlock();
                    startPath(pathToPos6, TIMEOUT_PATH_6);
                    startTpsWait();
                    state = AutoState.PATH_6;
                }
                break;

            case PATH_6:
                // Drive back to Shoot from Stack_2
                // Spin up shooter while returning
                if (shooterReadyOrTimedOut()) {
                    intake.intake(1, 1);
                }

                if (pathComplete()) {
                    stopIntakeAndBlock();
                    enterShootState();
                    state = AutoState.SHOOT_3;
                }
                break;

            case SHOOT_3:
                // Stopped at Shoot - fire Stack_2 balls
                runShootLogic();

                if (shootComplete()) {
                    stopIntakeAndBlock();
                    startPath(pathToPos7, TIMEOUT_PATH_7);
                    startTpsWait();
                    state = AutoState.PATH_7;
                }
                break;

            case PATH_7:
                // Drive to OverFlow
                if (shooterReadyOrTimedOut()) {
                    intake.intake(1, 1);
                    robotHardware.release();
                } else {
                    robotHardware.block();
                }

                if (pathComplete()) {
                    stopIntakeAndBlock();
                    startPath(pathToPos8, TIMEOUT_PATH_8);
                    startTpsWait();
                    state = AutoState.PATH_8;
                }
                break;

            case PATH_8:
                // Drive back to Shoot from OverFlow
                if (shooterReadyOrTimedOut()) {
                    intake.intake(1, 1);
                }

                if (pathComplete()) {
                    stopIntakeAndBlock();
                    enterShootState();
                    state = AutoState.SHOOT_4;
                }
                break;

            case SHOOT_4:
                // Stopped at Shoot - fire OverFlow balls
                runShootLogic();

                if (shootComplete()) {
                    stopIntakeAndBlock();
                    startPath(pathToPos9, TIMEOUT_DONE);
                    state = AutoState.DONE;
                }
                break;

            case DONE:
                break;
        }

        // ── Telemetry ────────────────────────────────────────────────────────
        Pose goalPose = getGoalPose();

        double dxGoal = goalPose.getX() - currentPose.getX();
        double dyGoal = goalPose.getY() - currentPose.getY();
        double distanceInches = Math.hypot(dxGoal, dyGoal);

        telemetry.addData("State", state);
        telemetry.addData("Path Elapsed (s)", "%.1f / %.1f", getRuntime() - pathStartTime, currentTimeout);
        telemetry.addData("Last Transition", lastTransitionWasTimeout ? "TIMEOUT" : "Normal");
        telemetry.addData("Follower Busy", follower.isBusy());

        if (state == AutoState.SHOOT_1 || state == AutoState.SHOOT_2
                || state == AutoState.SHOOT_3 || state == AutoState.SHOOT_4) {
            telemetry.addData("Shoot Timer (s)", "%.2f / %.2f",
                    shootTimer.seconds(), SHOOT_DURATION_SECONDS);
        }

        telemetry.addLine("─── Shooter ───");
        telemetry.addData("Distance (CM)", "%.1f", distCM);
        telemetry.addData("Target TPS", "%.0f", Outtake.target);
        telemetry.addData("Current TPS", "%.0f", Outtake.currentTPS);
        telemetry.addData("Shooter At Speed", isShooterAtSpeed());
        telemetry.addData("TPS Timeout Fired", tpsTimeoutFired);

        telemetry.addLine("─── Hood ───");
        telemetry.addData("Last Hood Cmd", "%.3f", RobotHardware.lastCommandedHood);
        telemetry.addData("Filtered Dist CM", "%.2f", RobotHardware.filteredDistanceCM);

        telemetry.addLine("─── Turret ───");
        telemetry.addData("Relative Angle", "%.1f°", Turret.relativeAngleDeg);
        telemetry.addData("Desired Servo", "%.3f", Turret.desiredServo);
        telemetry.addData("Current Servo", "%.3f", Turret.currentServo);

        telemetry.addLine("─── Goal / Pose ───");
        telemetry.addData("Goal X", "%.2f", goalPose.getX());
        telemetry.addData("Goal Y", "%.2f", goalPose.getY());
        telemetry.addData("Distance (in)", "%.2f", distanceInches);

        telemetry.addLine("─── Robot Pose ───");
        telemetry.addData("X", "%.2f", currentPose.getX());
        telemetry.addData("Y", "%.2f", currentPose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));

        telemetry.update();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Stop
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void stop() {
        PoseStorage.currentPose = follower.getPose();

        outtake.stopOuttake();
        outtake.updatePIDF();
        stopIntakeAndBlock();
        robotHardware.reset_all();
    }
}