package org.firstinspires.ftc.teamcode.wrappers;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDFMotor {

    public PIDController controller;
    public double p,i,d, f;
    public DcMotorEx motor;

    public final double ticksInDegree = 4005.558676 / 360.0;
    public Telemetry tele;

    public int target;
    public int increment;
    public PIDFMotor(DcMotorEx motor, Telemetry tele, int increment) {
        this.motor=motor;
        this.p=0.1;
        this.i=0;
        this.d=0.0001;
        this.f=0.22;
        this.increment = increment;
        this.tele = tele;
        this.target = 0;
        controller = new PIDController(p, i, d);
    }

    public void reset()
    {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void moveTo(int targetPos, float stickPower) {
        controller.setPID(p, i, d);
        int pos = motor.getCurrentPosition();
        int tolerance = 10;

        tele.addData("lPos: ", pos);
        tele.addData("lTarget: ", targetPos);
        tele.update();

        if(stickPower!=0 || Math.abs(pos-targetPos)<=tolerance) {
            motor.setPower(stickPower);
            return;
        }

        double pid = controller.calculate(pos, targetPos);
        double ff = Math.cos(Math.toRadians(targetPos / ticksInDegree)) * f;

        double power = pid + ff;

        motor.setPower(power);
    }

    public void moveToPower(int targetPos, double pow, boolean stopFinal) {
        controller.setPID(p,i,d);
        int pos = motor.getCurrentPosition();
        if(pos > 0) return;

        int tolerance = 5;

        if(stopFinal && Math.abs(pos - targetPos) >= tolerance)
        {
            motor.setPower(0);
            return;
        }

        double pid = controller.calculate(pos, targetPos);

        double ff = Math.cos(Math.toRadians(targetPos / ticksInDegree)) * f;


        double power = pid + ff;

        motor.setPower(power);
        tele.addData("pos: ", pos);
        tele.addData("target: ", targetPos);
        tele.update();
    }

    public DcMotorEx getMotor() {return motor;}

    public int getPos() {
        return motor.getCurrentPosition();
    }

    public void setPower(double power)
    {
        motor.setPower(power);
    }

    public void calibratePos(boolean up, boolean down) {
        int curPos = motor.getCurrentPosition();
        if(up) moveTo(curPos+increment, 0);
        if(down) moveTo(curPos-increment, 0);
        tele.addData("pos", curPos);
    }


}