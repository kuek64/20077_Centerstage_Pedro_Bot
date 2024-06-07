package org.firstinspires.ftc.teamcode.subsystem;

/*
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;


public class PresetSubsystem {
    private ClawSubsystem claw;
    private LiftSubsystem lift;
    private GearRotationSubsystem gear;

    public PresetSubsystem(ClawSubsystem clawSubsystem, LiftSubsystem liftSubsystem, GearRotationSubsystem gearRotationSubsystem) {
        this.claw = clawSubsystem;
        this.lift = liftSubsystem;
        this.gear = gearRotationSubsystem;
    }


    //------------------------------ Start Sequence ------------------------------//

    public Action GearStartPos() {
        return new SequentialAction(
                gear.startGear(),
                gear.waitForGear(),
                gear.stopGear(),
                gear.resetGear(),
                new SleepAction(.1),
                gear.wheelServo_Activated()
        );
    }

    public Action LiftStartPos() {
        return new SequentialAction(
                lift.stopLift(),
                lift.resetLift()
        );
    }

    /*public Action ClawStartPos() {
        return new SequentialAction(
                claw.closeClaws(),
                claw.groundClawPos()
        );
    }

    //------------------------------ Scoring Sequence ------------------------------//

    public Action ScoringPos() {
        return new ParallelAction(
                ClawScoringPos(),
                GearScoringPos(),
                LiftScoringPos()
        );
    }

    public Action GearScoringPos() {
        return new ParallelAction(
                gear.scoringGear(),
                gear.wheelServo_Deactivated()
                //gear.waitForGear(),
                //gear.stopGear()
        );
    }

    public Action LiftScoringPos() {
        return new ParallelAction(
                lift.liftExtend_Scoring()
                //lift.waitForLift(),
                //lift.stopLift()
        );
    }

    public Action ClawScoringPos() {
        return new SequentialAction(
                claw.scoringClaw()
        );
    }

    //------------------------------ Ground after Scoring Sequence ------------------------------//

    public Action GroundPos() {
        return new ParallelAction(
                LiftGroundPos(),
                GearGroundPos(),
                ClawGroundPos()
        );
    }

    public Action GearGroundPos() {
        return new ParallelAction(
                gear.groundGear(),
                gear.wheelServo_Activated()
        );
    }

    public Action LiftGroundPos() {
        return new SequentialAction(
                lift.liftRetract_Scoring()
        );
    }

    public Action ClawGroundPos() {
        return new SequentialAction(
                claw.groundClaw(),
                claw.openClaws()
        );
    }

    //------------------------------ White Stack Sequence------------------------------//
    public Action WhiteStack() {
        return new SequentialAction(
                WhiteStackStart(),
                new SleepAction(.1),
                WhiteStackEnd()
        );
    }

    /* White Stack Start */
/*
    public Action WhiteStackStart() {
        return new SequentialAction(
                //GearWhiteStackStart(),
                ClawWhiteStackStart(),
                LiftWhiteStackStart()
        );
    }

    public Action GearWhiteStackStart() {
        return new SequentialAction(
                //gear.stopGear(),
                //gear.wheelServo_Activated()
        );
    }

    public Action LiftWhiteStackStart() {
        return new SequentialAction(
                lift.liftExtend_Stack(),
                lift.waitForLift(),
                lift.stopLift()
        );
    }

    public Action ClawWhiteStackStart() {
        return new SequentialAction(
                claw.whiteGroundClaw(),
                claw.openClaws()
        );
    }

    /* White Stack End
    public Action WhiteStackEnd() {
        return new SequentialAction(
                ClawWhiteStackEnd(),
                new SleepAction(.25),
                LiftWhiteStackEnd(),
                new SleepAction(1.5)
        );
    }

    public Action GearWhiteStackEnd() {
        return new SequentialAction(
                gear.wheelServo_Activated()
        );
    }

    public Action LiftWhiteStackEnd() {
        return new SequentialAction(
                lift.liftRetract_Stack()
        );
    }

    public Action ClawWhiteStackEnd() {
        return new SequentialAction(
                claw.closeClaws()
        );
    }


    //------------------------------ Scoring Sequence ------------------------------//


    //------------------------------ Ground after White Scoring Sequence ------------------------------//

    public Action WhiteGroundPos() {
        return new ParallelAction(
                LiftWhiteGroundPos(),
                GearWhiteGroundPos(),
                ClawWhiteGroundPos()
        );
    }

    public Action GearWhiteGroundPos() {
        return new ParallelAction(
                gear.whiteGroundGear(),
                gear.wheelServo_Deactivated()
        );
    }

    public Action LiftWhiteGroundPos() {
        return new SequentialAction(
                lift.liftRetract_WhiteScoring()
        );
    }

    public Action ClawWhiteGroundPos() {
        return new SequentialAction(
                claw.groundClaw(),
                claw.openClaws()
        );
    }*/
//}

