package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.util.SingleRunAction;


public class ClawSubsystem {

    private Servo pivot = null;
    private Servo clawL = null;
    private Servo clawR = null;
    double closedL = 0.33;
    double closedR = 0.37;
    double openL = 0.45;//.42
    double openR = 0.25;//.28
    double groundClaw = 0.815;
    double scoringClaw = 0.25;
    double whiteGroundClaw = 0.85;
    double whiteScoringClaw = 0.75; //.725

    public SingleRunAction closeRClaw, closeLClaw, closeClaws, openLClaw, openRClaw, openClaws, groundClawPos, scoringClawPos, whiteGroundClawPos;

    public ClawSubsystem(HardwareMap hardwareMap) {
        pivot = hardwareMap.get(Servo.class, "pivot");
        clawL = hardwareMap.get(Servo.class, "clawL");
        clawR = hardwareMap.get(Servo.class, "clawR");

        closeLClaw = new SingleRunAction(() ->clawL.setPosition(closedL));
        closeRClaw = new SingleRunAction(() ->clawR.setPosition(closedR));
        closeClaws = new SingleRunAction(() ->clawR.setPosition(closedR));
        openLClaw = new SingleRunAction(() -> {
            clawR.setPosition(closedR); clawL.setPosition(closedL);
        });
        openRClaw = new SingleRunAction(() ->clawR.setPosition(closedR));
        openClaws = new SingleRunAction(() ->clawR.setPosition(closedR));
        groundClawPos = new SingleRunAction(() -> clawR.setPosition(closedR));
        scoringClawPos = new SingleRunAction(() -> clawR.setPosition(closedR));
        whiteGroundClawPos = new SingleRunAction(() -> clawR.setPosition(closedR));
    }
}