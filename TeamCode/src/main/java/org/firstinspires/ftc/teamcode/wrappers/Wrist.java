package org.firstinspires.ftc.teamcode.wrappers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    public Servo wrist;

    public static double PICKUP = 0.1394, DROP = 0.6694, HIGH = 0.8294;
    public static double HIGH_RUNG = 0.63, SPECIMEN = 0.5094;

    public Wrist(HardwareMap map) {
        wrist = map.servo.get("wrist");
    }

    public void setPosition(double pos) {
        wrist.setPosition(pos);
    }


    public class WristHighRung implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(HIGH_RUNG);
            return false;
        }
    }

    public Action highRung() {
        return new WristHighRung();
    }


    public class WristPickup implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(PICKUP);
            return false;
        }
    }
    public Action wristPickup() {
        return new WristPickup();
    }

    public class WristDrop implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(DROP);
            return false;
        }
    }
    public Action wristDrop() {
        return new WristDrop();
    }

    public class WristHigh implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(HIGH);
            return false;
        }
    }
    public Action wristHigh() {
        return new WristHigh();
    }

    public class WristSpecimen implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(SPECIMEN);
            return false;
        }
    }

    public Action wristSpecimen() {
        return new WristSpecimen();
    }

}
