package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.opModes.subClasses.Outtake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Intake;

@Autonomous(name = "Blue Side Close", group = "Blue")
public class bluesideClose extends OpMode {

    private Follower        follower;
    private Intake intake;
    private Outtake outtake;


    private enum AutoState {
        PATH_1,
        PATH_2,
        PATH_3,
        PATH_4,
        DONE
    }

    private AutoState state = AutoState.PATH_1;


    // Where the robot starts on the field
    private final Pose startPose = new Pose(21, 121, Math.toRadians(143));

    // Shooting Position
    private final Pose Shoot     = new Pose(60, 80, Math.toRadians(130));

    // 2nd stack
        private final Pose Stack_1     = new Pose(17, 82, Math.toRadians(177));


    private PathChain pathToPos1;
    private PathChain pathToPos2;
    private PathChain pathToPos3;

    public void buildPaths() {
        // Path 1: Straight line from start to shooting
        pathToPos1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, Shoot))
                .setLinearHeadingInterpolation(startPose.getHeading(), Shoot.getHeading())
                .build();

        // Path 2: Straight shooting to stack 2
        pathToPos2 = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, Stack_1))
                .setConstantHeadingInterpolation(Stack_1.getHeading())
                .build();

        pathToPos3 = follower.pathBuilder()
                .addPath(new BezierLine(Stack_1, Shoot))
                .setLinearHeadingInterpolation(Stack_1.getHeading(), Shoot.getHeading())
                .build();
    }


    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setStartingPose(startPose);

        outtake = new Outtake();
        intake = new Intake();

        outtake.init(hardwareMap);
        intake.init(hardwareMap);

        telemetry.addLine("Initialized. Ready to start.");
        telemetry.update();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        // Kick off the first path
        follower.followPath(pathToPos1);
        //outtake.startOuttaking(850, 1500);
        state = AutoState.PATH_1;
    }

    @Override
    public void loop() {
        follower.update();

        switch (state) {
            case PATH_1:
                // Intake runs during path 1
                intake.intake(1, 1);
                if (!follower.isBusy()) {
                    // Arrived at pose1 — stop intake and start path 2
                    intake.intakeStop();
                    follower.followPath(pathToPos2, true);
                    state = AutoState.PATH_2;
                }
                break;

            case PATH_2:
                // Intake off during path 2
                intake.intakeStop();
                if (!follower.isBusy()) {
                    state = AutoState.PATH_3;
                }
                break;

            case PATH_3:
                intake.intakeStop();
                follower.followPath(pathToPos3);
                if (!follower.isBusy()) {
                    state = AutoState.DONE;
                }
                break;

            case DONE:
                intake.intakeStop();
        }

        // Debugging telemetry on Driver Hub
        telemetry.addData("state",   state);
        telemetry.addData("x",       follower.getPose().getX());
        telemetry.addData("y",       follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.intakeStop();
    }
}