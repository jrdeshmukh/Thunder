package org.firstinspires.ftc.teamcode.wrappers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Mechanism {
    public Servo wristL, wristR, claw, spin;
    public static double CLAW_OPEN = 0, CLAW_CLOSE = 1;
    public static double SPIN_90 = 0.44, SPIN_0 = 0.8;
    public static double WRIST_FLAT = 0.42, DEGREE_TO_POS = -240;
    public static double WRIST_UP = 90, WRIST_OUT = 0, WRIST_DOWN = -90, WRIST_SCORE = 180, MATCH_SLIDE = 270;
    public static double SLIDE_TICKS_HEIGHT = 460;
    public static double WORM_START = 1462;
    public double wormPickup = 500;
    public boolean pickup = false, matchSlide = false;
    public static double wristCurrent = 0.5, CHOSEN_ANGLE = WRIST_UP, lastWristPos = 0.4;
    public Worm worm;
    public Slide slide;

    public Mechanism(HardwareMap hardwareMap) {
         wristL = hardwareMap.servo.get("wristL");
         wristR = hardwareMap.servo.get("wristR");
         claw = hardwareMap.servo.get("claw");
         spin = hardwareMap.servo.get("spin");
         spin.setPosition(SPIN_0);
         worm = new Worm(hardwareMap);
         slide = new Slide(hardwareMap, worm);
    }

    public double getWristAngle() {
        return (wristL.getPosition() - WRIST_FLAT) * DEGREE_TO_POS;
    }

    public double calcWristAngle(double targetAngleDegrees) {
        return ((targetAngleDegrees-worm.getAngle())/DEGREE_TO_POS) + WRIST_FLAT;
    }

    public void setWorm(double power) {
        pickup = false;
        worm.setPower(power);
    }

    public class UpdateAction implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            update();
            return true;
        }
    }

    public Action updateAction() {
        return new UpdateAction();
    }

    public void update() {
        wormPickup = worm.calcNeededPos(Math.toDegrees(Math.asin(662.619/(1865+slide.slide.getCurrentPosition()))));
        wristCurrent = ((CHOSEN_ANGLE-worm.getAngle())/DEGREE_TO_POS) + WRIST_FLAT;
        if(matchSlide) {
            setWrist(0.1);
            return;
        }
        setWrist(wristCurrent);
    }

    public void setChosenAngle(double chosenAngle) {
        matchSlide = chosenAngle==MATCH_SLIDE;
        CHOSEN_ANGLE = chosenAngle;
    }

    public void setSpin(double pos) {
        double actualPos = pos;
        if(pos < 0) {
            actualPos = pos + 1;
        }
        else if(pos>1){
            actualPos = pos -1;
        }
        spin.setPosition(actualPos);
    }

    public void setClaw(double pos) {
        claw.setPosition(pos);
    }

    public Action clawClose() {
       return new InstantAction(() -> setClaw(Mechanism.CLAW_CLOSE));
    }

    public Action clawOpen() {
        return  new InstantAction(() -> setClaw(Mechanism.CLAW_OPEN));
    }

    public Action wristPickup() {
        return new InstantAction(() -> setWrist(Mechanism.WRIST_DOWN));
    }

    public void setWrist(double posL) {
        if(lastWristPos==posL) {
            return;
        }
        wristL.setPosition(posL);
        wristR.setPosition(1-posL);
        lastWristPos = posL;
    }

    public void setPickup(boolean pickup) {
        this.pickup = pickup;
    }

    public double getWristPos() {
        return wristL.getPosition();
    }



}
