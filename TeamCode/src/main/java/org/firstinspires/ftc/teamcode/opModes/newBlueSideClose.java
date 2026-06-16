package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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

    // ── Goal target in FIELD coordinates ─────────────────────────────────────
    // Field system:
    // (0,0) bottom-left
    // +x right
    // +y up
    //
    // If you later decide the true target is the exact top-left corner,
    // change GOAL_Y to 144 and retest.
    public static double GOAL_X = 0;
    public static double GOAL_Y = 136.8;

    // Optional live trim if needed
    public static double GOAL_OFFSET_X = 0;
    public static double GOAL_OFFSET_Y = 0;

    // ── Path timeout constants (seconds) ─────────────────────────────────────
    private static final double TIMEOUT_PATH_1 = 4.0;  // Start → Shoot (preload)
    private static final double TIMEOUT_PATH_2 = 3.0;  // Shoot → Stack_1
    private static final double TIMEOUT_PATH_3 = 3.0;  // Stack_1 → Shoot
    private static final double TIMEOUT_PATH_4 = 4.0;  // Shoot → Stack_2 approach
    private static final double TIMEOUT_PATH_5 = 2.0;  // Stack_2 → EatStack_2
    private static final double TIMEOUT_PATH_6 = 4.0;  // EatStack_2 → Shoot
    private static final double TIMEOUT_PATH_7 = 3.0;  // Shoot → OverFlow
    private static final double TIMEOUT_PATH_8 = 3.0;  // OverFlow → Shoot
    private static final double TIMEOUT_DONE   = 3.0;  // Shoot → Park

    // ── TPS spin-up timeout ──────────────────────────────────────────────────
    private static final double TPS_SPINUP_TIMEOUT  = 2.0;
    private static final double TPS_READY_THRESHOLD = 0.95; // 95% of target TPS

    // ── Timeout tracking ─────────────────────────────────────────────────────
    private double pathStartTime = 0;
    private double currentTimeout = 0;
    private boolean lastTransitionWasTimeout = false;

    private double tpsWaitStartTime = 0;
    private boolean tpsTimeoutFired = false;

    // ── State machine ────────────────────────────────────────────────────────
    private enum AutoState {
        PATH_1, PATH_2, PATH_3, PATH_4,
        PATH_5, PATH_6, PATH_7, PATH_8,
        DONE
    }

    private AutoState state = AutoState.PATH_1;

    // ── Field poses ──────────────────────────────────────────────────────────
    // You may still want to retune these headings slightly on-field now that
    // localisation is corrected, but the structure remains the same.
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

    /**
     * Starts a path and stamps the timeout clock.
     */
    private void startPath(PathChain path, double timeoutSeconds) {
        follower.followPath(path);
        pathStartTime = getRuntime();
        currentTimeout = timeoutSeconds;
        lastTransitionWasTimeout = false;
    }

    private boolean pathTimedOut() {
        return (getRuntime() - pathStartTime) > currentTimeout;
    }

    /**
     * Returns true when path finishes naturally OR timeout expires.
     */
    private boolean pathComplete() {
        if (!follower.isBusy()) return true;

        if (pathTimedOut()) {
            lastTransitionWasTimeout = true;
            return true;
        }

        return false;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Shooter spin-up helpers
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * Call once when entering a shooter-wait section.
     */
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

    /**
     * Returns true when shooter is at speed OR timeout expires.
     * Use in state logic only.
     */
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
    // Goal / targeting helpers
    // ─────────────────────────────────────────────────────────────────────────

    private Pose getGoalPose() {
        return new Pose(GOAL_X + GOAL_OFFSET_X, GOAL_Y + GOAL_OFFSET_Y, 0);
    }

    private void updateGoalTarget() {
        turret.setPose(getGoalPose());
    }

    /**
     * Distance from robot pose to goal in CM.
     * Pedro pose is in inches, regressions use CM.
     */
    private double distanceToGoalCM() {
        Pose robotPose = follower.getPose();
        Pose goalPose = getGoalPose();

        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();

        return Math.hypot(dx, dy) * 2.54;
    }

    /**
     * Preview only — no hardware commands.
     */
    private double previewTurretServo() {
        Pose robotPose = follower.getPose();
        Pose goalPose = getGoalPose();

        double dx = goalPose.getX() - robotPose.getX();
        double dy = goalPose.getY() - robotPose.getY();

        double targetAngle = Math.atan2(dy, dx);
        double robotHeading = robotPose.getHeading();

        double relativeAngle = targetAngle - robotHeading;
        relativeAngle = Math.atan2(Math.sin(relativeAngle), Math.cos(relativeAngle));

        // Must match working Turret.java mapping
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

        // Set field target now so preview and runtime both match
        updateGoalTarget();

        // No servo / motor movement here
        telemetry.addLine("Initialised. Ready to start.");
        telemetry.addData("Goal X", "%.2f", getGoalPose().getX());
        telemetry.addData("Goal Y", "%.2f", getGoalPose().getY());
        telemetry.update();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Init loop — preview only, no hardware commands
    // ─────────────────────────────────────────────────────────────────────────

    @Override
    public void init_loop() {
        updateGoalTarget();

        double distCM = distanceToGoalCM();

        // Safe: only updates target variable
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
        telemetry.addLine("");
        telemetry.addLine("Servos inactive until START");
        telemetry.update();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Start — first legal moment to move hardware
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

        // Turret tracks goal every loop from live field pose
        turret.aimTurret(currentPose);

        // Shooter + hood regression from live field distance
        double distCM = distanceToGoalCM();
        outtake.linearRegression(distCM);
        robotHardware.linearHoodRegression(distCM);

        // Must run every loop to drive shooter motors
        outtake.updatePIDF();

        switch (state) {

            case PATH_1:
                // Start → Shoot
                if (shooterReadyOrTimedOut()) {
                    intake.intake(1, 1);
                    robotHardware.release();
                }

                if (pathComplete()) {
                    startPath(pathToPos2, TIMEOUT_PATH_2);
                    startTpsWait();
                    state = AutoState.PATH_2;
                }
                break;

            case PATH_2:
                // Shoot → Stack_1
                if (shooterReadyOrTimedOut()) {
                    robotHardware.release();
                } else {
                    robotHardware.block();
                }

                if (pathComplete()) {
                    stopIntakeAndBlock();
                    startPath(pathToPos3, TIMEOUT_PATH_3);
                    state = AutoState.PATH_3;
                }
                break;

            case PATH_3:
                // Stack_1 → Shoot
                if (pathComplete()) {
                    startPath(pathToPos4, TIMEOUT_PATH_4);
                    state = AutoState.PATH_4;
                }
                break;

            case PATH_4:
                // Shoot → Stack_2 approach
                if (pathComplete()) {
                    startPath(pathToPos5, TIMEOUT_PATH_5);
                    startTpsWait();
                    state = AutoState.PATH_5;
                }
                break;

            case PATH_5:
                // Stack_2 → EatStack_2
                if (shooterReadyOrTimedOut()) {
                    intake.intake(1, 1);
                    robotHardware.release();
                } else {
                    robotHardware.block();
                }

                if (pathComplete()) {
                    stopIntakeAndBlock();
                    startPath(pathToPos6, TIMEOUT_PATH_6);
                    state = AutoState.PATH_6;
                }
                break;

            case PATH_6:
                // EatStack_2 → Shoot
                if (pathComplete()) {
                    startPath(pathToPos7, TIMEOUT_PATH_7);
                    startTpsWait();
                    state = AutoState.PATH_7;
                }
                break;

            case PATH_7:
                // Shoot → OverFlow
                if (shooterReadyOrTimedOut()) {
                    intake.intake(1, 1);
                    robotHardware.release();
                } else {
                    robotHardware.block();
                }

                if (pathComplete()) {
                    stopIntakeAndBlock();
                    startPath(pathToPos8, TIMEOUT_PATH_8);
                    state = AutoState.PATH_8;
                }
                break;

            case PATH_8:
                // OverFlow → Shoot, then park path
                if (pathComplete()) {
                    stopIntakeAndBlock();
                    startPath(pathToPos9, TIMEOUT_DONE);
                    state = AutoState.DONE;
                }
                break;

            case DONE:
                // Park path running
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

        telemetry.addLine("─── Shooter ───");
        telemetry.addData("Distance (CM)", "%.1f", distCM);
        telemetry.addData("Target TPS", "%.0f", Outtake.target);
        telemetry.addData("Current TPS", "%.0f", Outtake.currentTPS);
        telemetry.addData("Shooter At Speed", isShooterAtSpeed());
        telemetry.addData("Spin-up Timed Out", hasSpinUpTimedOut());
        telemetry.addData("TPS Timeout Fired", tpsTimeoutFired);

        telemetry.addLine("─── Hood ───");
        telemetry.addData("Last Hood Cmd", "%.3f", RobotHardware.lastCommandedHood);
        telemetry.addData("Filtered Dist CM", "%.2f", RobotHardware.filteredDistanceCM);
        telemetry.addData("Hood Slope", "%.6f", RobotHardware.hoodSlope);
        telemetry.addData("Hood Intercept", "%.6f", RobotHardware.hoodIntercept);

        telemetry.addLine("─── Turret ───");
        telemetry.addData("Relative Angle", "%.1f°", Turret.relativeAngleDeg);
        telemetry.addData("Desired Servo", "%.3f", Turret.desiredServo);
        telemetry.addData("Current Servo", "%.3f", Turret.currentServo);

        telemetry.addLine("─── Goal / Pose ───");
        telemetry.addData("Goal X", "%.2f", goalPose.getX());
        telemetry.addData("Goal Y", "%.2f", goalPose.getY());
        telemetry.addData("dx Goal", "%.2f", dxGoal);
        telemetry.addData("dy Goal", "%.2f", dyGoal);
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
        outtake.updatePIDF(); // zero motors cleanly
        stopIntakeAndBlock();
        robotHardware.reset_all();
    }
}