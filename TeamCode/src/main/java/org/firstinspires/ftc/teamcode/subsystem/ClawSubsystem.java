package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.util.SingleRunAction;

public class ClawSubsystem {

    public Servo clawL, clawR, pivot;

    public Telemetry telemetryA;

    public VoltageSensor controlHubVoltageSensor;

    public SingleRunAction openClaws, openLClaw, openRClaw;

    public double deltaTimeSeconds, outtakeWristDirection, outtakeWristOffset, intakeArmTargetPosition, outtakeArmTargetPosition, intakeArmOutPosition, outtakePreviousStaticPosition, pickUpAdjustDirection;

    public int intakeState, outtakeState, intakeArmTargetState, outtakeArmTargetState, liftTargetPosition, liftPresetTargetPosition, transferState, intakeSpeed, outtakeSpeed;

    public ClawSubsystem(HardwareMap hardwareMap) {
        clawL = hardwareMap.get(Servo.class, "clawL");
        clawR = hardwareMap.get(Servo.class, "clawR");
        pivot = hardwareMap.get(Servo.class, "pivot");

        openClaws = new SingleRunAction(() -> {
            clawR.setPosition(0);
            clawL.setPosition(0);
        });

        openLClaw = new SingleRunAction(() -> {
            clawL.setPosition(0);
        });

        openRClaw = new SingleRunAction(() -> {
            clawL.setPosition(0);
        });
    }
}