package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.FollowPathAction;
import org.firstinspires.ftc.teamcode.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.GearRotationSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.PresetSubsystem;

import java.util.concurrent.TimeUnit;


@Config
@Autonomous(name = "Blue Close 2 + 2",group = "Blue")
public final class Blue_Close2_2 extends LinearOpMode {

    private HuskyLens huskyLens;
    private ClawSubsystem claw;
    private GearRotationSubsystem gear;
    private PresetSubsystem presets;
    private LiftSubsystem lift;


    private Servo clawL, clawR, pivot;
    private Follower follower;

    private Pose startPose = new Pose(-62, 12, 0);
    private Pose yellowScoringPose1 = new Pose(-36, 30, Math.toRadians(270));
    private Pose yellowScoringPose2 = new Pose(-30, 22, Math.toRadians(270));
    private Pose yellowScoringPose3 = new Pose(-36, 8, Math.toRadians(270));
    private Pose yellowScoringOverridePose1 = new Pose(-42, 45, Math.toRadians(270));
    private Pose yellowScoringOverridePose2 = new Pose(-36, 45, Math.toRadians(270));
    private Pose yellowScoringOverridePose3 = new Pose(-27, 45, Math.toRadians(270));
    private Pose driveToWhitePose1 = new Pose(-42, 51.5, Math.toRadians(270));//-38
    private Pose driveToWhitePose2 = new Pose(-36, 51.5, Math.toRadians(270));
    private Pose driveToWhitePose3 = new Pose(-27, 51.5, Math.toRadians(270));
    private Pose inTrussPose = new Pose(-60, 24, Math.toRadians(270));
    private Pose throughWhiteTruss = new Pose(-60, -24, Math.toRadians(270));
    private Pose whiteTrussPose = new Pose(-35.5,-37, Math.toRadians(270));
    private Pose whiteScoringPose = new Pose(-60, 24, Math.toRadians(270));
    private Pose parkingPose = new Pose(-40.5, 52.75, Math.toRadians(90));

    Path purplePath1 = new Path(
            new BezierLine(new Point(startPose),
                    new Point(yellowScoringPose1)));

    Path purplePath2 = new Path(
            new BezierLine(new Point(startPose),
            new Point(yellowScoringPose2)));

    Path purplePath3 = new Path(
            new BezierLine(new Point(startPose),
            new Point(yellowScoringPose3)));

    Path yellowPath1 = new Path(
            new BezierLine(new Point(yellowScoringPose1),
            new Point(driveToWhitePose1)));

    Path yellowPath2 = new Path(
            new BezierLine(new Point(yellowScoringPose2),
            new Point(driveToWhitePose2)));

    Path yellowPath3 = new Path(
            new BezierLine(new Point(yellowScoringPose3),
            new Point(driveToWhitePose3)));

    Path towhiteCycle1Path1 = new Path(
            new BezierCurve(new Point(driveToWhitePose1),
            new Point(inTrussPose),
            new Point(throughWhiteTruss),
            new Point(whiteTrussPose)));

    Path towhiteCycle1Path2 = new Path(
            new BezierCurve(new Point(driveToWhitePose2),
            new Point(inTrussPose),
            new Point(throughWhiteTruss),
            new Point(whiteTrussPose)));

    Path towhiteCycle1Path3 = new Path(
            new BezierCurve(new Point(driveToWhitePose3),
            new Point(inTrussPose),
            new Point(throughWhiteTruss),
            new Point(whiteTrussPose)));

    Path backwhiteCycle1Path1 = new Path(
            new BezierCurve(new Point(whiteTrussPose),
                    new Point(throughWhiteTruss),
                    new Point(inTrussPose),
                    new Point(yellowScoringPose3)));

    Path parkingPath = new Path(
            new BezierLine(new Point(yellowScoringPose3),
                    new Point(throughWhiteTruss)));


    @Override
    public void runOpMode() {
        ClawSubsystem claw = new ClawSubsystem(hardwareMap);
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        clawL = hardwareMap.get(Servo.class, "clawL");

        clawL.setPosition(0.5);

        //Huskylens Setup
        Deadline rateLimit = new Deadline(1, TimeUnit.SECONDS);
        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);


        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {

            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());// this gives you the data
                telemetry.addData("location?", blocks[i].x);// this gives you just x

                //----------------------------1----------------------------\\
                    Actions.runBlocking(
                            new SequentialAction(
                                    new ParallelAction(
                                            new FollowPathAction(follower, purplePath1, false),
                                            new SleepAction(0.5)

                                    ),
                                    //claw.openLClaw(),
                                    //new SleepAction(0.35),
                                    new ParallelAction(
                                            new ParallelAction(
                                                    //presets.ScoringPos(),
                                                    new FollowPathAction(follower, yellowPath1, false),
                                                    new SleepAction(0.5)
                                            )
                                            //claw.closeLClaw()
                                    ),
                                    //new SleepAction(.1),
                                    //claw.openRClaw(),
                                    //new SleepAction(.25),
                                    new ParallelAction(
                                            new SequentialAction(
                                                    //presets.GroundPos(),
                                                    //claw.openClaws(),
                                                    //claw.whiteGroundClaw()
                                            ),
                                            new FollowPathAction(follower, towhiteCycle1Path1, false),
                                            new SleepAction(0.5)
                                    ),
                                    new SequentialAction(
                                            //presets.WhiteStack(),
                                            new FollowPathAction(follower, backwhiteCycle1Path1, false),
                                            new SleepAction(0.5)
                                            //presets.WhiteScoringPos()
                                            )
                                    ),

                                    new SequentialAction(
                                            new SleepAction(.1),
                                            claw.openClaws(),
                                            new SleepAction(.25),
                                            new FollowPathAction(follower, parkingPath, false),
                                            presets.WhiteGroundPos()
                                    )
                            );
                    sleep(400000);
                }


                //----------------------------2----------------------------\\
                /*if (blocks[i].x > 100 && blocks[i].x < 200 && blocks[i].id == 2 && blocks[i].y < 200) {
                    Actions.runBlocking(
                            new SequentialAction(
                                    new ParallelAction(
                                            presets.StartPos(),
                                            purpleAction2
                                    ),
                                    new SleepAction(0.1),
                                    claw.openLClaw(),
                                    new SleepAction(0.25),
                                    new ParallelAction(
                                            new SequentialAction(
                                                    presets.ScoringPos(),
                                                    yellowScoringAction2
                                            ),
                                            claw.closeLClaw()
                                    )
                            )
                    );

                    camera.alignToTag(2);

                    if(!camera.targetFound){
                        Actions.runBlocking(
                                yellowScoringOverrideAction2
                        );
                    }

                    Actions.runBlocking(
                            new SequentialAction(
                                    new SleepAction(.1),
                                    claw.openRClaw(),
                                    new SleepAction(.25),
                                    new ParallelAction(
                                            new SequentialAction(
                                                    presets.GroundPos(),
                                                    claw.openClaws(),
                                                    claw.whiteGroundClaw()
                                            ),
                                            driveToWhiteAction2
                                    ),
                                    new SequentialAction(
                                            presets.WhiteStack(),
                                            new SleepAction(0.25),
                                            whiteTrussAction,
                                            new ParallelAction(
                                                    whiteScoringAction,
                                                    presets.WhiteScoringPos()
                                            )
                                    ),

                                    new SequentialAction(
                                            new SleepAction(.25),
                                            claw.openClaws(),
                                            new SleepAction(.25),
                                            parkingAction,
                                            presets.WhiteGroundPos()
                                    )
                            )
                    );
                    sleep(400000);
                }


                //----------------------------3---------------------------\\
                if (blocks[i].x > 210 && blocks[i].id == 2 && blocks[i].y < 200) {
                    Actions.runBlocking(
                            new SequentialAction(
                                    new ParallelAction(
                                            presets.StartPos(),
                                            purpleAction3
                                    ),
                                    new SleepAction(0.1),
                                    claw.openLClaw(),
                                    new SleepAction(0.25),
                                    new ParallelAction(
                                            new SequentialAction(
                                                    presets.ScoringPos(),
                                                    yellowScoringAction3
                                            ),
                                            claw.closeLClaw()
                                    )
                            )
                    );

                    camera.alignToTag(3);

                    if(!camera.targetFound){
                        Actions.runBlocking(
                                yellowScoringOverrideAction3
                        );
                    }

                    Actions.runBlocking(
                            new SequentialAction(
                                    new SleepAction(.1),
                                    claw.openRClaw(),
                                    new SleepAction(.25),
                                    new ParallelAction(
                                            new SequentialAction(
                                                    presets.GroundPos(),
                                                    claw.openClaws(),
                                                    claw.whiteGroundClaw()
                                            ),
                                            driveToWhiteAction3
                                    ),
                                    new SequentialAction(
                                            presets.WhiteStack(),
                                            new SleepAction(0.25),
                                            whiteTrussAction,
                                            new ParallelAction(
                                                    whiteScoringAction,
                                                    presets.WhiteScoringPos()
                                            )
                                    ),

                                    new SequentialAction(
                                            new SleepAction(.1),
                                            claw.openClaws(),
                                            new SleepAction(.25),
                                            parkingAction,
                                            presets.WhiteGroundPos()
                                    )
                            )
                    );
                    sleep(400000);
                }
            }*/
        }

    }
}