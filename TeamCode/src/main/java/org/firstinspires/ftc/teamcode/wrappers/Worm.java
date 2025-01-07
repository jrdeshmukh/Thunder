package org.firstinspires.ftc.teamcode.wrappers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Config
public class Worm {
    public DcMotor worm, copycat;
    public static boolean overrideLimit = false;
    private final PIDController controller, ccontroller;
    public static double p = 0.0075, i = 0, d = 0.00003;
    public static int offset = 0;

    public static final int BOTTOM = 0;
    public static final int HIGH_BASKET = 0;
    public static final int LOW_BASKET = 0;
    public static final int HIGH_RUNG = 0;
    public boolean active = false;
    public static double curPow = 0;
    public double wormPow=0, copycatPow=0;
    public double targetPosition;
    public boolean humanControl = true;
    public static double TICKS_PER_DEGREE = 29.44444;
    public boolean pickup = false;
    public double lastPower = 15, tol = 0.01;
    EeshMechanism mechanism;


    public TouchSensor dontBreakIntake;
    public TouchSensor dontBreakMotor;
    public Worm(HardwareMap map, EeshMechanism mechanism) {
        worm = map.get(DcMotorEx.class, "worm");
        copycat = map.get(DcMotorEx.class, "copycat");

        worm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        copycat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        targetPosition = mechanism.wormCurrent;

        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);
        ccontroller = new PIDController(p, i, d);
        this.mechanism = mechanism;

        dontBreakIntake = map.touchSensor.get("dontBreakIntake");
        dontBreakMotor =  map.touchSensor.get("dontBreakMotor");
    }

    public void runToPos(int targetPosition) {
        humanControl = false;
        this.targetPosition = targetPosition;
        if(this.targetPosition > 1500) this.targetPosition = 1500;
    }

    public double getAngle() {
        return mechanism.wormCurrent/TICKS_PER_DEGREE + 54.714;
    }

    public double calcNeededPos(double angle) {
        return (angle-54.714)*TICKS_PER_DEGREE;
    }

    public void setPow2() {
        if(humanControl) {
            //if(Math.abs(lastPower)>tol) {
                worm.setPower(0);
                copycat.setPower(0);
            //}
            lastPower = 0;
            return;
        }
        int pos = mechanism.wormCurrent;
        wormPow = controller.calculate(pos, targetPosition);
        copycatPow = ccontroller.calculate(mechanism.copycatCurrent, targetPosition);

        //boolean dontBreakIntakeLim = dontBreakIntake.isPressed() && curPow < 0;
        //boolean dontBreakMotorLim = dontBreakMotor.isPressed() && curPow > 0;

        //if(!dontBreakIntakeLim && !dontBreakMotorLim) {
            worm.setPower(wormPow);
            copycat.setPower(copycatPow);
            lastPower = wormPow;
        //}
    }

    public class ResetEncoder implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            worm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            copycat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            worm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            copycat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            targetPosition = 0;
            return false;
        }
    }


    public class WaitUntilClose implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return Math.abs(mechanism.wormCurrent-targetPosition)>30;
        }
    }

    public Action waitUntilDone() {
        return new WaitUntilClose();
    }

    public Action resetEncoders() {
        return new ResetEncoder();
    }


    public Action autoMove(int pos) {
        return new InstantAction(() -> this.runToPos(pos));
    }



    public class SetPow implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setPow2();
            return true;
        }
    };

    public Action setPow() {
        return new SetPow();
    }







    public void setPower(double power) {
        humanControl = true;
        /*if((power<0&&worm.getCurrentPosition()<200) || (power>0&&worm.getCurrentPosition()>3000)) {
            worm.setPower(0);
            copycat.setPower(0);
            return;
        }
        if(power<0 && worm.getCurrentPosition()<800) {
            worm.setPower(0.5*power);
            copycat.setPower(0.5*power);
            return;
        }
        if(power>0&&worm.getCurrentPosition()>2500) {
            worm.setPower(0.2*power);
            copycat.setPower(0.2*power);
        }*/

        if(dontBreakIntake.isPressed() && power < 0)
        {
            worm.setPower(0);
            copycat.setPower(0);
            lastPower = 0;
        }
        else if(mechanism.wormCurrent <= EeshMechanism.WORMPICKUPPOS && power < 0)
        {
            worm.setPower(0);
            copycat.setPower(0);
            lastPower = 0;
        }
        else if(dontBreakMotor.isPressed() && power > 0)
        {
            worm.setPower(0);
            copycat.setPower(0);
        }
        else if(mechanism.wormCurrent >= EeshMechanism.WORMMAX && power > 0)
        {
            worm.setPower(0);
            copycat.setPower(0);
        }
        else {
            if(Math.abs(lastPower-power)>tol) {
                worm.setPower(power);
                copycat.setPower(power);
            }
            lastPower = power;
        }



    }


}
