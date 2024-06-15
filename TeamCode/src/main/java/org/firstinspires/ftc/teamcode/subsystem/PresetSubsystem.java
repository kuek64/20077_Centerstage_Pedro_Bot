package org.firstinspires.ftc.teamcode.subsystem;

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

    public void GearStartPos() {
                gear.startGear();
                gear.stopGear();
                gear.resetGear();
                new SleepAction(.1);
                gear.wheelServo_Activated();
    }

    public void LiftStartPos() {
                lift.stopLift();
                lift.resetLift();
    }

    public void ClawStartPos() {
                claw.closeClaws();
                claw.groundClawPos();
    }

    //------------------------------ Scoring Sequence ------------------------------//

    public void ScoringPos() {
                ClawScoringPos();
                GearScoringPos();
                LiftScoringPos();
    }

    public void GearScoringPos() {
                gear.scoringGear();
                gear.wheelServo_Deactivated();
    }

    public void LiftScoringPos() {
                lift.liftExtend_Scoring();
    }

    public void ClawScoringPos() {
                claw.scoringClawPos();
    }

    //------------------------------ Ground after Scoring Sequence ------------------------------//

    public void GroundPos() {
                LiftGroundPos();
                GearGroundPos();
                ClawGroundPos();
    }

    public void GearGroundPos() {
                gear.groundGear();
                gear.wheelServo_Activated();
    }

    public void LiftGroundPos() {
                lift.liftRetract_Scoring();

    }

    public void ClawGroundPos() {
                claw.groundClawPos();
                claw.openClaws();
    }

    //------------------------------ White Stack Sequence------------------------------//
    public void WhiteStack() {
                WhiteStackStart();
                new SleepAction(.1);
                WhiteStackEnd();
    }

    /* White Stack Start */

    public void WhiteStackStart() {
                ClawWhiteStackStart();
                LiftWhiteStackStart();
    }

    public void LiftWhiteStackStart() {
                lift.liftExtend_Stack();
                lift.stopLift();
    }

    public void ClawWhiteStackStart() {
                claw.whiteGroundClawPos();
                claw.openClaws();
    }

    //------------------------------ White Stack End------------------------------//
    public void WhiteStackEnd() {
                ClawWhiteStackEnd();
                new SleepAction(.25);
                LiftWhiteStackEnd();
                new SleepAction(1.5);
    }

    public void GearWhiteStackEnd() {
                gear.wheelServo_Activated();
    }

    public void LiftWhiteStackEnd() {
                lift.liftRetract_Stack();
    }

    public void ClawWhiteStackEnd() {
                claw.closeClaws();
    }


    //------------------------------ Scoring Sequence -------------------------------------------------//

    //-------------------- Ground after White Scoring Sequence --------------------//

    public void WhiteGroundPos() {
                LiftWhiteGroundPos();
                ClawWhiteGroundPos();
    }



    public void LiftWhiteGroundPos() {
                lift.liftRetract_WhiteScoring();
    }

    public void ClawWhiteGroundPos() {
                claw.groundClawPos();
                claw.openClaws();
    }
}