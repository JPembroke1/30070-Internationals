package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.field.Line;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.opModes.subClasses.Outtake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Intake;

import java.net.HttpURLConnection;

@Autonomous(name = "Blue Side Close", group = "Blue")
public class bluesideClose extends OpMode {

    private Follower        follower;
    private Intake intake;
    private Outtake outtake;
    private Turret turret;


    private enum AutoState {
        PATH_1,
        PATH_2,
        PATH_3,
        PATH_4,
        PATH_5,
        PATH_6,
        PATH_7,
        PATH_8,
        DONE
    }

    private AutoState state = AutoState.PATH_1;


    // Where the robot starts on the field
    private final Pose startPose = new Pose(21, 121, Math.toRadians(143));

    // Shooting Position
    private final Pose Shoot     = new Pose(60, 80, Math.toRadians(177));

    // 2nd stack
    private final Pose Stack_1     = new Pose(17, 82, Math.toRadians(177));
    private final Pose Stack_2 = new Pose(40, 56, Math.toRadians(170));
    private final Pose EatStack_2 = new Pose(15, 60, Math.toRadians(170));
    private final Pose OverFlow = new Pose(13, 60, Math.toRadians(150));
    private final Pose End = new Pose(50, 70, Math.toRadians(0));


    private PathChain pathToPos1;
    private PathChain pathToPos2;
    private PathChain pathToPos3;
    private PathChain pathToPos4;
    private PathChain pathToPos5;
    private PathChain pathToPos6;
    private PathChain pathToPos7;
    private PathChain pathToPos8;
    private PathChain pathToPos9;

    public void buildPaths() {
        // Path 1: Straight line from start to shooting
        pathToPos1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, Shoot))
                .setLinearHeadingInterpolation(startPose.getHeading(), Shoot.getHeading())
                .build();

        // Path 2: Straight shooting to stack 2
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


    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setStartingPose(startPose);

        outtake = new Outtake();
        intake = new Intake();
        turret = new Turret();

        outtake.init(hardwareMap);
        intake.init(hardwareMap);
        turret.init(hardwareMap);

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
        turret.aimTurret();

        switch (state) {
            case PATH_1:
                // Intake runs during path 1
                outtake.startOuttaking(850);

                if (Outtake.currentTPS > 1500) {
                    intake.intake(1, 1);
                    if (!follower.isBusy()) {
                        // Arrived at pose1 — stop intake and start path 2
                        follower.followPath(pathToPos2);
                        state = AutoState.PATH_2;
                    }
                    break;

                }

            case PATH_2:
                // Intake off during path 2
                if (!follower.isBusy()) {
                    state = AutoState.PATH_3;
                }
                break;

            case PATH_3:
                intake.intakeStop();
                follower.followPath(pathToPos3);
                if (!follower.isBusy()) {
                    state = AutoState.PATH_4;
                }
                break;

            case PATH_4:
                follower.followPath(pathToPos4);
                if (!follower.isBusy()) {
                    state = AutoState.PATH_5;
                }
                break;

            case PATH_5:
                follower.followPath(pathToPos5);
                if (!follower.isBusy()) {
                    state = AutoState.PATH_6;
                }
                break;

            case PATH_6:
                follower.followPath(pathToPos6);
                if (!follower.isBusy()) {
                    state = AutoState.PATH_7;
                }
                break;

            case PATH_7:
                follower.followPath(pathToPos7);
                if (!follower.isBusy()) {
                    state = AutoState.PATH_8;
                }
                break;

            case PATH_8:
                follower.followPath(pathToPos8);
                if (!follower.isBusy()) {
                    state = AutoState.DONE;
                }
                break;

            case DONE:
                intake.intakeStop();
                follower.followPath(pathToPos9);
                break;
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