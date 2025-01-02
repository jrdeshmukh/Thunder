package org.firstinspires.ftc.teamcode.wrappers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config()
public class Slide {
    public DcMotorEx slide;
    private final PIDController controller;
    public static double p = 0.008, i = 0, d = 0.00005, f = 0.00005;

    public static final int BOTTOM = 0;
    public static final int HIGH_BASKET = 3150;
    public static final int LOW_BASKET = 1875;
    public static final int HIGH_RUNG = 1013;
    public static final int APPROACH_HANG = 1520;
    public static final int SCORE_SPECIMEN = 870;
    public static double curPow = 0;
    public static final int MAX_FLAT_POS = 1700; //1722 ticks = 37 inches
    public static final int MAX_UP_POS = 3200; //highest it can physically go
    public static int fullExtension = 1722;
    public double lastPower = 15, tol = 0.01;
    Worm worm;


    public double targetPosition = 0;
    public boolean humanControl = false;

    public Slide(HardwareMap map, Worm worm) {
        slide = map.get(DcMotorEx.class, "slide");
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        this.worm = worm;

        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);
    }

    public void runToPos(int targetPosition) {
        this.targetPosition = targetPosition;
        humanControl = false;
    }

    public void update() {
        fullExtension = (int) Math.abs(Math.min(MAX_UP_POS, (MAX_FLAT_POS/Math.cos(Math.toRadians(worm.getAngle())))));
    }

    public int getFullExtension() {
        fullExtension = (int) Math.abs(Math.min(MAX_UP_POS, MAX_FLAT_POS/Math.cos(Math.toRadians(worm.getAngle()))));
        return fullExtension;
    }

    public void setPow2() {
        int pos = EeshMechanism.slideCurrent;
        double ff = pos * f;
        if(humanControl) {
            if(Math.abs(lastPower-curPow)>tol) {
                slide.setPower(curPow);
            }
            lastPower = curPow;
            return;
        }
        double pid = controller.calculate(pos, Math.min(getFullExtension(), targetPosition));
        curPow = pid + ff;
        if(Math.abs(lastPower-curPow)>tol) {
            slide.setPower(curPow);
        }
    }

    public class ResetEncoder implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            targetPosition = 0;
            return false;
        }
    }

    public Action resetEncoders() {
        return new ResetEncoder();
    }



    public class SetPow implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            int pos = EeshMechanism.slideCurrent;
            double pid = controller.calculate(pos, targetPosition);
            double ff = pos * f;
            curPow = pid + ff;
            if(Math.abs(lastPower-curPow)>tol) {
                slide.setPower(curPow);
            }
            return true;
        }
    }

    public Action setPow() {
        return new SetPow();
    }

    public class HighRung implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            targetPosition = Slide.HIGH_RUNG;
            return false;
        }
    }

    public Action highRung() {
        return new HighRung();
    }



    public Action autoMove(int pos) {
        return new InstantAction(() -> runToPos(pos));
    }


    public class WaitUntilClose implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return Math.abs(EeshMechanism.slideCurrent-targetPosition)>50;
        }
    }

    public Action waitUntilDone() {
        return new WaitUntilClose();
    }

    public class LiftHigh implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            targetPosition = Slide.HIGH_BASKET;
            return false;
        }
    }


    public Action liftHigh() {
        return new LiftHigh();
    }

    public class LiftLow implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
             targetPosition = Slide.LOW_BASKET;
             return false;
        }
    }
    public Action liftLow() {
        return new LiftLow();
    }

    public class LiftBottom implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            humanControl = false;
            targetPosition = Slide.BOTTOM;
            return false;
        }
    }
    public Action liftBottom() {
        return new LiftBottom();
    }

    public void setPower(double power) {
        humanControl = true;
        if(EeshMechanism.slideCurrent < getFullExtension() || power < 0) {
            slide.setPower(power);
        }
        else{
            slide.setPower(p*(fullExtension-EeshMechanism.slideCurrent));
        }
    }


}
