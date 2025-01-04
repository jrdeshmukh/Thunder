package org.firstinspires.ftc.teamcode.wrappers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Worm {
    public DcMotorEx worm, copycat;
    public static boolean overrideLimit = false;
    private final PIDController controller;
    public final double p = 0.0075, i = 0, d = 0.00003;
    public static int offset = 0;

    public static final int BOTTOM = 0;
    public static final int HIGH_BASKET = 0;
    public static final int LOW_BASKET = 0;
    public static final int HIGH_RUNG = 0;
    public boolean active = false;
    public static double curPow = 0;
    public double targetPosition;
    public boolean humanControl = true;
    public static double TICKS_PER_DEGREE = 29.44444;
    public boolean pickup = false;
    public double lastPower = 15, tol = 0.01;



    public TouchSensor dontBreakIntake;
    public TouchSensor dontBreakMotor;
    public Worm(HardwareMap map) {
        worm = map.get(DcMotorEx.class, "worm");
        copycat = map.get(DcMotorEx.class, "copycat");

        worm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        copycat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        targetPosition = EeshMechanism.wormCurrent;

        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);

        dontBreakIntake = map.touchSensor.get("dontBreakIntake");
        dontBreakMotor =  map.touchSensor.get("dontBreakMotor");
    }

    public void runToPos(int targetPosition) {
        humanControl = false;
        this.targetPosition = targetPosition;
        if(this.targetPosition > 1500) this.targetPosition = 1500;
    }

    public double getAngle() {
        return EeshMechanism.wormCurrent/TICKS_PER_DEGREE + 54.714;
    }

    public double calcNeededPos(double angle) {
        return angle*TICKS_PER_DEGREE;
    }

    public void setPow2() {
        if(humanControl) {
            if(Math.abs(lastPower)>tol) {
                worm.setPower(0);
                copycat.setPower(0);
            }
            lastPower = 0;
            return;
        }
        int pos = EeshMechanism.wormCurrent;
        curPow = controller.calculate(pos, targetPosition);

        double copycatPow = controller.calculate(EeshMechanism.copycatCurrent, targetPosition);

        boolean dontBreakIntakeLim = dontBreakIntake.isPressed() && curPow < 0;
        boolean dontBreakMotorLim = dontBreakMotor.isPressed() && curPow > 0;

        if(!dontBreakIntakeLim && !dontBreakMotorLim) {
            worm.setPower(curPow);
            copycat.setPower(copycatPow);
            lastPower = curPow;
        }
    }

    public void setPow3() {
        int pos = EeshMechanism.wormCurrent;
        curPow = controller.calculate(pos, targetPosition);

        boolean dontBreakIntakeLim = dontBreakIntake.isPressed() && curPow < 0;
        boolean dontBreakMotorLim = dontBreakMotor.isPressed() && curPow > 0;

        if(Math.abs(lastPower-curPow)>tol) {
            worm.setPower(curPow);
            copycat.setPower(curPow);
        }
        lastPower = curPow;
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
            return Math.abs(EeshMechanism.wormCurrent-targetPosition)>20;
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
            int pos = EeshMechanism.wormCurrent;
            double pid = controller.calculate(pos, targetPosition);

            boolean dontBreakIntakeLim = dontBreakIntake.isPressed() && pid < 0;
            boolean dontBreakMotorLim = dontBreakMotor.isPressed() && pid > 0;

            if(!dontBreakIntakeLim && !dontBreakMotorLim)
            {
                if(Math.abs(lastPower-curPow)>tol) {
                    worm.setPower(curPow);
                    copycat.setPower(curPow);
                }
                lastPower = curPow;
            }

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
        }
        else if(EeshMechanism.wormCurrent <= EeshMechanism.WORMPICKUPPOS && power < 0)
        {
            worm.setPower(0);
            copycat.setPower(0);
        }
        else if(dontBreakMotor.isPressed() && power > 0)
        {
            worm.setPower(0);
            copycat.setPower(0);
        }
        else if(EeshMechanism.wormCurrent >= EeshMechanism.WORMMAX && power > 0)
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
