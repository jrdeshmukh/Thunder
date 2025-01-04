package org.firstinspires.ftc.teamcode.wrappers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class EeshMechanism {


    public static int WORMPICKUPPOS = -9100; //minimum
    public static double WORMMAX = 1500;
    public static double WRISTDOWNPOSSUBMERSIBLE = 0.4572;

    public static double WRISTPICKUPLOW = 0.5344;

    public static double WRISTHOVER = 0.6167;
    public static double WRISTHOVERHIGH = 0.7567;
    public static double WRIST_PICKUP_ANGLE = -48.6952991170, WRIST_DROP_ANGLE = 135, WRIST_SPECIMEN_ANGLE = 90;
    public boolean angle = true;

    public static int INTAKE_DISTANCE = 1;


    public Servo wristL, wristR;

    public CRServo intake;
    public static double WRIST_FLAT = 0.576, DEGREE_TO_POS = 280.6361;
    public static double WRIST_UP = 90, WRIST_OUT = 0, WRIST_DOWN = -90, WRIST_SCORE = 180, MATCH_SLIDE = 270;
    public static double SLIDE_TICKS_HEIGHT = 460;
    public static double WORM_START = 1462;
    public double wormPickup = 500;
    public boolean pickup = false, matchSlide = false;
    public static double wristCurrent = 0.5, CHOSEN_ANGLE = WRIST_UP, lastWristPos = 0.4;
    public static int wormCurrent = 0, slideCurrent = 0, copycatCurrent = 0;
    public Worm worm;
    public Slide slide;

    public EeshMechanism(HardwareMap hardwareMap) {
        wristL = hardwareMap.servo.get("wristL");
        wristR = hardwareMap.servo.get("wristR");
        intake = hardwareMap.crservo.get("intake");
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

    public void setIntake(double power) {
        intake.setPower(power);
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
        wormCurrent = worm.worm.getCurrentPosition();
        copycatCurrent = worm.copycat.getCurrentPosition();
        slideCurrent = slide.slide.getCurrentPosition();
        wormPickup = worm.calcNeededPos(Math.toDegrees(Math.asin(662.619/(1865+EeshMechanism.slideCurrent))));
        wristCurrent = (CHOSEN_ANGLE-worm.getAngle())/DEGREE_TO_POS + WRIST_FLAT;
        if(matchSlide) {
            setWrist(0.1);
            return;
        }
        if(angle) {
            setWrist(wristCurrent);
            angle = true;
        };
    }

    public void setChosenAngle(double chosenAngle) {
        matchSlide = chosenAngle==MATCH_SLIDE;
        CHOSEN_ANGLE = chosenAngle;
        angle = true;
    }




    public Action wristPickup() {
        return new InstantAction(() -> setWrist(Mechanism.WRIST_DOWN));
    }

    public void setWrist(double posL) {
        //if(Math.abs(lastWristPos-posL)<0.1) {
          //  return;
        //}
        wristL.setPosition(posL);
        wristR.setPosition(1-posL);
        lastWristPos = posL;
        angle = false;
    }

    public void setPickup(boolean pickup) {
        this.pickup = pickup;
    }

    public double getWristPos() {
        return lastWristPos;
    }



}
