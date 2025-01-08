package org.firstinspires.ftc.teamcode.wrappers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class EeshMechanism {

    public static double X = 2000, Y = 705;
    public static int WORMPICKUPPOS = -9100; //minimum
    public static double WORMMAX = 1500;
    public static double WRISTDOWNPOSSUBMERSIBLE = 0.4572;

    public static double WRISTPICKUPLOW = 0.5344;

    public static double WRISTHOVER = 0.6167;
    public static double WRISTHOVERHIGH = 0.7567;
    public static double WRIST_PICKUP_ANGLE = -25.695299117, WRIST_DROP_ANGLE = 135, WRIST_SPECIMEN_ANGLE = 90;
    public boolean angle = true;

    public static int INTAKE_DISTANCE = 1;


    public Servo wristL, wristR;

    public CRServo intake;
    public static double WRIST_FLAT = 0.421, DEGREE_TO_POS = -261.39;
    public static double WRIST_UP = 90, WRIST_OUT = 0, WRIST_DOWN = -90, WRIST_SCORE = 180, MATCH_SLIDE = 270;
    public static double SLIDE_TICKS_HEIGHT = 460;
    public static double WORM_START = 1462;
    public double wormPickup = -800;
    public boolean pickup = false, matchSlide = false;
    public static double wristCurrent = 0.5, CHOSEN_ANGLE = WRIST_UP, lastWristPos = 0.4;
    public int wormCurrent = 0, slideCurrent = 0, copycatCurrent = 0;
    DcMotor frontLeft, frontRight, backRight;
    public Worm worm;
    public Slide slide;

    public EeshMechanism(HardwareMap hardwareMap) {
        wristL = hardwareMap.servo.get("wristL");
        wristR = hardwareMap.servo.get("wristR");
        intake = hardwareMap.crservo.get("intake");
        frontLeft = hardwareMap.dcMotor.get("leftFront");
        frontRight = hardwareMap.dcMotor.get("leftRear");
        backRight = hardwareMap.dcMotor.get("rightRear");
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        worm = new Worm(hardwareMap, this);
        slide = new Slide(hardwareMap, worm, this);
    }

    public double getWristAngle() {
        return (lastWristPos - WRIST_FLAT) * DEGREE_TO_POS;
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
    public Action startIntake() {return new InstantAction(() -> intake.setPower(1));}
    public Action drop() {return new InstantAction(() -> intake.setPower(-1));}
    public Action dunk() {return new InstantAction(this::dunkIntake);}

    public void dunkIntake(){
        setChosenAngle(300);
        intake.setPower(-1);
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
        copycatCurrent = -backRight.getCurrentPosition();
        wormCurrent = frontRight.getCurrentPosition();
        slideCurrent = -frontLeft.getCurrentPosition();
        wormPickup = worm.calcNeededPos(Math.toDegrees(Math.asin(Y/(X+this.slideCurrent))));
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

    public Action setAngleAction(double a) {
        return new InstantAction(() -> setChosenAngle(a));
    }




    public Action wristPickup() {
        return new InstantAction(() -> setWrist(WRIST_DOWN));
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
