package org.firstinspires.ftc.teamcode.wrappers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    public Servo claw;

    public static final double OPEN = 0.75, CLOSE = 0.3;

    public Claw(HardwareMap map) {
        claw = map.servo.get("claw");
    }

    public void setPosition(double pos) {
        claw.setPosition(pos);
    }

    public class ClawClose implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.setPosition(CLOSE);
            return false;
        }
    }
    public Action close() {
        return new ClawClose();
    }
    public class ClawOpen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.setPosition(OPEN);
            return false;
        }
    }

    public Action open() {
        return new ClawOpen();
    }

}
