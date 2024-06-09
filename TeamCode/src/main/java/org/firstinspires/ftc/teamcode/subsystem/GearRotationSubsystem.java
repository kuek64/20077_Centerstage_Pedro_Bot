package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class GearRotationSubsystem {
    private DcMotorEx gear;
    private Servo wheelServo;

    public GearRotationSubsystem(HardwareMap hardwareMap) {
        gear = hardwareMap.get(DcMotorEx.class, "gear");
        gear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gear.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelServo = hardwareMap.get(Servo.class, "WheelServo");
    }

    //------------------------------Ground Position------------------------------//
    public void groundGear() {
                gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                gear.setTargetPosition(-760);
                gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gear.setPower(0.5);
    }

    public void whiteGroundGear() {
                gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                gear.setTargetPosition(-348); //-300
                gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gear.setPower(0.5);
    }

    //------------------------------Scoring Position------------------------------//
    public void scoringGear() {
                gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                gear.setTargetPosition(760);
                gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gear.setPower(0.5);
    }

    public void whiteScoringGear() {
                gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                gear.setTargetPosition(348); //300
                gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gear.setPower(0.5);
            }

    //------------------------------Start Position-------------------------------//
    public void startGear() {
                gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                gear.setTargetPosition(-500);
                gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gear.setPower(0.5);
    }

    //------------------------------End Position-------------------------------//
    public void endGear() {
                gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                gear.setTargetPosition(-500);
                gear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                gear.setPower(0.5);
    }

    //------------------------------ Wait for Gear -------------------------------//
    /*public Action waitForGear() {
        return new Action() {

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return gear.isBusy();
            }
        };
    }*/

    //------------------------------ Stop Gear -------------------------------//
    public void stopGear() {
                gear.setPower(0);
    }

    public void resetGear() {
                gear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //------------------------------ Wheel Servo for Stack -------------------------------//
    public void wheelServo_Activated() {
                wheelServo.setPosition(0.636); /* Top 2 Pixels | bigger # = lower */ //0.641
    }

    public void wheelServo_ActivatedFar() {
                wheelServo.setPosition(0.5725); /* Top 2 Pixels | bigger # = lower */ //0.641
    }

    public void wheelServo_Deactivated() {
                wheelServo.setPosition(0.85);
            }
}