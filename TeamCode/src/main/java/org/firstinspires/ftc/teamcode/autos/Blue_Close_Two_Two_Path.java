package org.firstinspires.ftc.teamcode.autos;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;

@Autonomous(name = "Blue Close 2+2 Path", group = "Blue")
public class Blue_Close_Two_Two_Path extends OpMode {

    private Timer pathTimer, opmodeTimer, scanTimer;
    private ClawSubsystem clawSubsystem;

    private String navigation;

    //Spike mark locations
    private Pose blueLeftSideLeftSpikeMark = new Pose(51, 30+72);
    private Pose blueLeftSideMiddleSpikeMark = new Pose(-30+72, 22+72);
    private Pose blueLeftSideRightSpikeMark = new Pose(-36+72, 8+72);

    //Backdrop zone locations
    private Pose blueLeftBackdrop = new Pose(36, 51.5+72);
    private Pose blueMiddleBackdrop = new Pose(-36+72, 51.5+72);
    private Pose blueRightBackdrop = new Pose(-27+72, 51.5+72);
    private Pose blueWhiteBackdrop = new Pose(46, 51.5+72, Math.toRadians(270));

    //Through Truss
    private Pose blueTopTruss = new Pose(12+8+2, 84);
    private Pose blueBottomTruss = new Pose(12+8+2, 36);

    // white pixel stack locations
    private Pose blueLeftStack = new Pose(-36+72+24, -37+72);
    private Pose blueMiddleStack = new Pose(-36+72+12, -37+72);
    private Pose blueRightStack = new Pose(-36+72, 12, Math.toRadians(270));

    private Pose spikeMarkGoalPose, initialBackdropGoalPose, firstCycleStackPose, firstCycleBackdropGoalPose, secondCycleStackPose, secondCycleBackdropGoalPose;

    // TODO: adjust this for each auto
    private Pose startPose = new Pose(-62+72, 12+72, 0);

    private Follower follower;

    private Path scoreSpikeMark, initialScoreOnBackdrop;
    private PathChain cycleStackTo, cycleStackBack;

    private int pathState;

    public void setBackdropGoalPose() {
        switch (navigation) {
            default:
            case "left":
                spikeMarkGoalPose = new Pose(blueLeftSideLeftSpikeMark.getX(), blueLeftSideLeftSpikeMark.getY(), Math.toRadians(270));
                initialBackdropGoalPose = new Pose(blueLeftBackdrop.getX(), blueLeftBackdrop.getY(), Math.toRadians(270));
                firstCycleBackdropGoalPose = new Pose(blueLeftBackdrop.getX(), blueLeftBackdrop.getY(), Math.toRadians(270));

                break;
            case "middle":
                spikeMarkGoalPose = new Pose(blueLeftSideMiddleSpikeMark.getX(), blueLeftSideMiddleSpikeMark.getY()+3, Math.toRadians(270));
                initialBackdropGoalPose = new Pose(blueMiddleBackdrop.getX(), blueMiddleBackdrop.getY(),Math.toRadians(270));
                firstCycleBackdropGoalPose = new Pose(blueMiddleBackdrop.getX(), blueMiddleBackdrop.getY(), Math.toRadians(270));
                break;
            case "right":
                spikeMarkGoalPose = new Pose(blueLeftSideRightSpikeMark.getX(), blueLeftSideRightSpikeMark.getY(), Math.toRadians(270));
                initialBackdropGoalPose = new Pose(blueRightBackdrop.getX(), blueRightBackdrop.getY(), Math.toRadians(270));
                firstCycleBackdropGoalPose = new Pose(blueRightBackdrop.getX(), blueRightBackdrop.getY(), Math.toRadians(270));
                break;
        }
    }

    public void buildPaths() {
        scoreSpikeMark = new Path(new BezierLine(new Point(startPose), new Point(spikeMarkGoalPose)));
        scoreSpikeMark.setConstantHeadingInterpolation(Math.toRadians(0));
        scoreSpikeMark.setPathEndTimeoutConstraint(0);

        initialScoreOnBackdrop = new Path(new BezierLine(new Point(spikeMarkGoalPose), new Point(initialBackdropGoalPose)));
        initialScoreOnBackdrop.setLinearHeadingInterpolation(spikeMarkGoalPose.getHeading(), initialBackdropGoalPose.getHeading());
        initialScoreOnBackdrop.setPathEndTimeoutConstraint(0);

        cycleStackTo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstCycleBackdropGoalPose), new Point(blueTopTruss)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .addPath(new BezierLine(new Point(blueTopTruss), new Point(blueBottomTruss)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .addPath(new BezierLine(new Point(blueBottomTruss), new Point(blueRightStack)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        cycleStackBack = follower.pathBuilder()
                .addPath(new BezierLine(new Point(blueRightStack), new Point(blueBottomTruss)))
                .setConstantHeadingInterpolation(blueWhiteBackdrop.getHeading())
                .addPath(new BezierLine(new Point(blueBottomTruss), new Point(blueTopTruss)))
                .setConstantHeadingInterpolation(blueWhiteBackdrop.getHeading())
                .addPath(new BezierLine(new Point(blueTopTruss), new Point(blueWhiteBackdrop)))
                .setConstantHeadingInterpolation(blueWhiteBackdrop.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10:
                follower.followPath(scoreSpikeMark);
                scoreSpikeMark.setLinearHeadingInterpolation(startPose.getHeading(), spikeMarkGoalPose.getHeading());
                setPathState(11);
                break;
            case 11:
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    follower.followPath(initialScoreOnBackdrop);
                    setPathState(13);
                }
                break;
            case 13:
                follower.holdPoint(new BezierPoint(initialScoreOnBackdrop.getLastControlPoint()), initialBackdropGoalPose.getHeading());
                setPathState(14);
                break;
            case 14:
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    setPathState(15);
                }
                break;
            case 15:
                if(!follower.isBusy()) {
                    follower.followPath(cycleStackTo, true);
                    setPathState(16);
                }
                break;
            case 16:
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    setPathState(17);
                }
                break;
            case 17:
                if(!follower.isBusy()) {
                    follower.followPath(cycleStackBack, true);
                    setPathState(18);
                }
                break;
            case 18:
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    setPathState(19);
                }
                break;
            case 19:
                if(!follower.isBusy()) {
                    follower.followPath(cycleStackTo, true);
                    setPathState(20);
                }
                break;
            case 20:
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    setPathState(21);
                }
                break;
            case 21:
                if(!follower.isBusy()) {
                    follower.followPath(cycleStackBack, true);
                    setPathState(22);
                }
                break;
            case 22:
                if (pathTimer.getElapsedTimeSeconds() > 3.5) {
                    setPathState(23);
                }
                break;
            case 23:
                if(!follower.isBusy()) {
                    follower.followPath(cycleStackBack, true);
                    setPathState(24);
                }
                break;
            case 24:
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    setPathState(25);
                }
                break;
            case 25:
                if(!follower.isBusy()) {
                    follower.followPath(cycleStackTo, true);
                    setPathState(26);
                }
                break;
            case 26:
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    setPathState(27);
                }
                break;
            case 27:
                if(!follower.isBusy()) {
                    follower.followPath(cycleStackBack, true);
                    setPathState(28);
                }
                break;
            case 28:
                if(opmodeTimer.getElapsedTimeSeconds() > 30) {
                    follower.breakFollowing();
                    setPathState(-1);
                }
                break;
        }
    }
    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
        autonomousPathUpdate();
    }

    @Override
    public void loop() {
        follower.update();

        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());

        //telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        scanTimer = new Timer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        scanTimer.resetTimer();

    }

    @Override
    public void init_loop() {
        if (scanTimer.getElapsedTime() > 750) {
            scanTimer.resetTimer(); }
    }

    @Override
    public void start() {
        navigation = "left";
        setBackdropGoalPose();
        buildPaths();
        opmodeTimer.resetTimer();
        setPathState(10);
    }

    @Override
    public void stop() {
    }
}