package org.firstinspires.ftc.teamcode.autos;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name = "Blue Close 2+2 Path", group = "Blue")
public class Blue_Close_Two_Two_Path extends OpMode {

    private Timer pathTimer, opmodeTimer, scanTimer;

    private String navigation;

    //Spike mark locations
    private Pose blueLeftSideLeftSpikeMark = new Pose(-36+72, 30+72);
    private Pose blueLeftSideMiddleSpikeMark = new Pose(-30+72, 22+72);
    private Pose blueLeftSideRightSpikeMark = new Pose(-36+72, 8+72);

    //Backdrop zone locations
    private Pose blueLeftBackdrop = new Pose(-42, 51.5+72);
    private Pose blueMiddleBackdrop = new Pose(-36, 51.5+72);
    private Pose blueRightBackdrop = new Pose(-27, 51.5+72);

    // white pixel stack locations
    private Pose blueOuterStack = new Pose(-35.5+72, -37+72);

    private Pose spikeMarkGoalPose, initialBackdropGoalPose, firstCycleStackPose, firstCycleBackdropGoalPose, secondCycleStackPose, secondCycleBackdropGoalPose;

    // TODO: adjust this for each auto
    private Pose startPose = new Pose(-62+72, 12+72, 0);

    private Follower follower;

    private Path scoreSpikeMark, initialScoreOnBackdrop;
    private PathChain firstCycleToStack, firstCycleStackGrab, firstCycleScoreOnBackdrop, secondCycleToStack, secondCycleStackGrab, secondCycleScoreOnBackdrop;

    private int pathState;

    public void setBackdropGoalPose() {
        switch (navigation) {
            default:
            case "left":
                spikeMarkGoalPose = new Pose(blueLeftSideLeftSpikeMark.getX(), blueLeftSideLeftSpikeMark.getY(), Math.toRadians(270));
                initialBackdropGoalPose = new Pose(blueLeftBackdrop.getX(), blueLeftBackdrop.getY(), Math.toRadians(270));
                firstCycleBackdropGoalPose = new Pose(blueRightBackdrop.getX(), blueRightBackdrop.getY(), Math.toRadians(270));
                secondCycleBackdropGoalPose = new Pose(blueRightBackdrop.getX(), blueRightBackdrop.getY(), Math.toRadians(270));
                break;
            case "middle":
                spikeMarkGoalPose = new Pose(blueLeftSideMiddleSpikeMark.getX(), blueLeftSideMiddleSpikeMark.getY()+3, Math.PI/2);
                initialBackdropGoalPose = new Pose(blueMiddleBackdrop.getX(), blueMiddleBackdrop.getY(),Math.toRadians(270));
                firstCycleBackdropGoalPose = new Pose(blueRightBackdrop.getX(), blueRightBackdrop.getY(), Math.toRadians(270));
                secondCycleBackdropGoalPose = new Pose(blueRightBackdrop.getX(), blueRightBackdrop.getY(), Math.toRadians(270));
                break;
            case "right":
                spikeMarkGoalPose = new Pose(blueLeftSideRightSpikeMark.getX(), blueLeftSideRightSpikeMark.getY(), Math.toRadians(270));
                initialBackdropGoalPose = new Pose(blueRightBackdrop.getX(), blueRightBackdrop.getY(), Math.toRadians(270));
                firstCycleBackdropGoalPose = new Pose(blueMiddleBackdrop.getX(), blueRightBackdrop.getY(), Math.toRadians(270));
                secondCycleBackdropGoalPose = new Pose(blueRightBackdrop.getX(), blueRightBackdrop.getY(), Math.toRadians(270));
                break;
        }
    }

    public void buildPaths() {
        Point scoreSpikeMarkMidPoint;
        double scoreSpikeMarkMidToSpikeDistance;
        switch (navigation) {
            default:
            case "left":
                scoreSpikeMarkMidPoint = new Point(144-131.5, 82, Point.CARTESIAN);
                break;
            case "middle":
                scoreSpikeMarkMidPoint = new Point(144-129.5, 106, Point.CARTESIAN);
                break;
            case "right":
                scoreSpikeMarkMidPoint = new Point(144-122.5, 99, Point.CARTESIAN);
                break;
        }
        scoreSpikeMarkMidToSpikeDistance = MathFunctions.distance(spikeMarkGoalPose, scoreSpikeMarkMidPoint);
        scoreSpikeMark = new Path(new BezierCurve(new Point(startPose), scoreSpikeMarkMidPoint, new Point(spikeMarkGoalPose.getX() + MathFunctions.getSign(scoreSpikeMarkMidPoint.getX() - spikeMarkGoalPose.getX()) * Math.abs(scoreSpikeMarkMidPoint.getX() - spikeMarkGoalPose.getX()) / scoreSpikeMarkMidToSpikeDistance, spikeMarkGoalPose.getY() + MathFunctions.getSign(scoreSpikeMarkMidPoint.getY() - spikeMarkGoalPose.getY()) * Math.abs(scoreSpikeMarkMidPoint.getY() - spikeMarkGoalPose.getY())/ scoreSpikeMarkMidToSpikeDistance, Point.CARTESIAN)));
        scoreSpikeMark.setConstantHeadingInterpolation(startPose.getHeading());
        scoreSpikeMark.setPathEndTimeoutConstraint(3);

        switch (navigation) {
            default:
            case "left":
                initialScoreOnBackdrop = new Path(new BezierCurve(scoreSpikeMark.getLastControlPoint(), new Point(144-135, 98, Point.CARTESIAN), new Point(initialBackdropGoalPose.getX(), 106, Point.CARTESIAN), new Point(initialBackdropGoalPose)));
                break;
            case "middle":
                initialScoreOnBackdrop = new Path(new BezierLine(scoreSpikeMark.getLastControlPoint(), new Point(initialBackdropGoalPose)));
                break;
            case "right":
                initialScoreOnBackdrop = new Path(new BezierCurve(scoreSpikeMark.getLastControlPoint(), new Point(scoreSpikeMark.getLastControlPoint().getX(), 98, Point.CARTESIAN), new Point(initialBackdropGoalPose.getX(), 109.5, Point.CARTESIAN), new Point(initialBackdropGoalPose)));
                break;
        }
        //initialScoreOnBackdrop.setConstantHeadingInterpolation(Math.PI * 1.5);
        initialScoreOnBackdrop.setLinearHeadingInterpolation(scoreSpikeMark.getEndTangent().getTheta(), Math.PI * 1.5, 0.5);
        initialScoreOnBackdrop.setPathEndTimeoutConstraint(2.5);

        switch (navigation) {
            default:
            case "left":
                firstCycleStackPose = new Pose(blueOuterStack.getX(), blueOuterStack.getY(), Math.toRadians(270));
                secondCycleStackPose = new Pose(blueOuterStack.getX(), blueOuterStack.getY(), Math.toRadians(270));
                break;
            case "middle":
                firstCycleStackPose = new Pose(blueOuterStack.getX(), blueOuterStack.getY(), Math.toRadians(270));
                secondCycleStackPose = new Pose(blueOuterStack.getX(), blueOuterStack.getY(), Math.toRadians(270));
                break;
            case "right":
                firstCycleStackPose = new Pose(blueOuterStack.getX(), blueOuterStack.getY(), Math.toRadians(270));
                secondCycleStackPose = new Pose(blueOuterStack.getX(), blueOuterStack.getY(), Math.toRadians(270));
                break;
        }

        firstCycleToStack = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(initialBackdropGoalPose), new Point(144-76.5, 106, Point.CARTESIAN), new Point(firstCycleStackPose.getX()+2, 79, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(firstCycleStackPose.getHeading())
                .addPath(new BezierLine(new Point(firstCycleStackPose.getX(), 79, Point.CARTESIAN), new Point(firstCycleStackPose.getX(), 23, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(firstCycleStackPose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        firstCycleStackGrab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstCycleStackPose.getX(), 32, Point.CARTESIAN), new Point(firstCycleStackPose)))
                .setConstantHeadingInterpolation(firstCycleStackPose.getHeading())
                .build();

        firstCycleScoreOnBackdrop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstCycleStackPose), new Point(firstCycleStackPose.getX(), 79, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(firstCycleStackPose.getHeading())
                .addPath(new BezierCurve(new Point(firstCycleStackPose.getX()+2, 79, Point.CARTESIAN), new Point(144-76.5, 106, Point.CARTESIAN), new Point(firstCycleBackdropGoalPose)))
                .setConstantHeadingInterpolation(Math.PI * 1.5)
                .setPathEndTimeoutConstraint(2.5)
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10:
                follower.followPath(scoreSpikeMark);
                setPathState(11);
                break;
            case 11:
                if (pathTimer.getElapsedTime() > 100) {
                    follower.followPath(initialScoreOnBackdrop);
                    setPathState(12);
                    break;
                }
            case 12:
                if (pathTimer.getElapsedTime() > 100) {
                    follower.followPath(firstCycleToStack);
                    setPathState(13);
                    break;
                }
            case 13:
                if (pathTimer.getElapsedTime() > 100) {
                    follower.followPath(firstCycleStackGrab);
                    setPathState(14);
                    break;
                }
            case 14:
                if (pathTimer.getElapsedTime() > 100) {
                    follower.followPath(firstCycleScoreOnBackdrop);
                    setPathState(15);
                    break;
                }
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

/**
 * 8==D
 */