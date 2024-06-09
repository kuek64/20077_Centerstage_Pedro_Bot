package org.firstinspires.ftc.teamcode.subsystem;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;


public class PresetSubsystem {
    public ClawSubsystem claw;
    public LiftSubsystem lift;
    public GearRotationSubsystem gear;

    public PresetSubsystem(ClawSubsystem clawSubsystem, LiftSubsystem liftSubsystem, GearRotationSubsystem gearRotationSubsystem) {
        this.claw = clawSubsystem;
        this.lift = liftSubsystem;
        this.gear = gearRotationSubsystem;
    }


    //------------------------------ Start Sequence ------------------------------//
    public void StartPos() {
        ClawStartPos();
        LiftStartPos();
        GearStartPos();
    }

    public void GearStartPos() {
        gear.startGear();
        gear.stopGear();
        gear.resetGear();
        gear.wheelServo_Activated();
    }

    public void LiftStartPos() {
        lift.stopLift();
        lift.resetLift();
    }

    public void ClawStartPos() {
        claw.closeClaws();
        claw.groundClaw();
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
        //lift.waitForLift(),
        //lift.stopLift()
    }

    public void ClawScoringPos() {
        claw.scoringClaw();
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
        claw.groundClaw();
        claw.openClaws();
    }
    
}

