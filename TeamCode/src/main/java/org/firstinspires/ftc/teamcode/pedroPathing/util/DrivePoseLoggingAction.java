package org.firstinspires.ftc.teamcode.pedroPathing.util;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

public class DrivePoseLoggingAction implements Action {
    String label;
    String message;
    static ElapsedTime autoStartTimer;
    static ElapsedTime lastLogEndTimer;
    Pose2d poseToLog;
    Follower follower;

    public DrivePoseLoggingAction(Follower follower, String label, boolean isStart) {
        this.follower = follower;
        this.label = label;
        if(isStart) {
            autoStartTimer = null;
        }
    }

    public DrivePoseLoggingAction(Follower follower, String label) {
        this.follower = follower;
        this.label = label;
    }

    public DrivePoseLoggingAction(Follower follower, String label, String message, boolean isStart) {
        this.follower = follower;
        this.label = label;
        this.message = message;

        if(isStart) {
            autoStartTimer = null;
        }
    }

    public DrivePoseLoggingAction(Follower follower, String label, String message) {
        this.follower = follower;
        this.label = label;
        this.message = message;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        if(autoStartTimer == null) {
            autoStartTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        }

        if(lastLogEndTimer == null) {
            lastLogEndTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        }

        lastLogEndTimer.reset();
        return false;
    }
}

