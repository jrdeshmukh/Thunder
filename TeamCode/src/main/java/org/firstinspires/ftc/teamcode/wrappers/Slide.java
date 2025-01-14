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
    public static double p = 0.018, i = 0, d = 0.0003, f = 0.00005;

    public static
    int BOTTOM = 0;
    public static  int HIGH_BASKET = 3150;
    public static  int LOW_BASKET = 1875;
    public static  int HIGH_RUNG = 1013;
    public boolean passed = false;
    public static  int APPROACH_HANG = 1520;
    public static  int SCORE_SPECIMEN = 870;
    public static double curPow = 0;
    public static  int MAX_FLAT_POS = 1090; //1722 ticks = 37 inches
    public static  int MAX_UP_POS = 3200; //highest it can physically go
    public static int fullExtension = 1722;
    public double lastPower = 15, tol = 0.01;
    Worm worm;


    public double targetPosition = 0;
    public boolean humanControl = true;
    EeshMechanism mechanism;

    public Slide(HardwareMap map, Worm worm, EeshMechanism mechanism) {
        slide = map.get(DcMotorEx.class, "slide");
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        this.worm = worm;

        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);
        this.mechanism = mechanism;
    }

    public void runToPos(int targetPosition) {
        this.targetPosition = targetPosition;
        passed = false;
        humanControl = false;
    }

    public void update() {
        fullExtension = (int) Math.abs(Math.min(MAX_UP_POS, ((MAX_FLAT_POS+2000)/Math.cos(Math.toRadians(worm.getAngle())))-2000));
    }

    public int getFullExtension() {
        fullExtension = (int) Math.abs(Math.min(MAX_UP_POS, MAX_FLAT_POS/Math.cos(Math.toRadians(worm.getAngle()))));
        return fullExtension;
    }

    public void setPow2() {
        int pos = mechanism.slideCurrent;
        double ff = (pos+1865) * f * Math.sin(Math.toRadians(worm.getAngle()));
        if(humanControl) {
            if(Math.abs(lastPower-ff)>tol) {
                slide.setPower(ff);
                lastPower = ff;
            }
            return;
        }
        double pid = controller.calculate(pos,targetPosition);
        curPow = pid + ff;
        if(Math.abs(lastPower-curPow)>tol) {
            slide.setPower(curPow);
        }
        lastPower = curPow;
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
            setPow2();
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
            return Math.abs(mechanism.slideCurrent-targetPosition)>70;
        }
    }

    public class StartDispensing implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return Math.abs(mechanism.slideCurrent-targetPosition)>200;
        }
    }

    public Action waitUntilDone() {
        return new WaitUntilClose();
    }

    public Action startDispensing() {
        return new StartDispensing();
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
        if(mechanism.slideCurrent > getFullExtension()) passed = true;
        if(power < 0) passed = false;
        if(!passed) {
            if(Math.abs(lastPower-power)>tol) {
                slide.setPower(power);
            }
            lastPower = power;
        }
        else{
            runToPos(getFullExtension());
            setPow2();
        }
    }


}
