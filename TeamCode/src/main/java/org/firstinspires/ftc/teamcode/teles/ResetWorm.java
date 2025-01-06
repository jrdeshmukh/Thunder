package org.firstinspires.ftc.teamcode.teles;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.wrappers.EeshMechanism;
import org.firstinspires.ftc.teamcode.wrappers.Worm;

@TeleOp
public class ResetWorm extends OpMode {
    private EeshMechanism mechanism;
    private TouchSensor limitSwitch;
    double pow = 0.6;
    DcMotor frontLeft, frontRight, backRight;




    @Override
    public void init() {
        Worm.overrideLimit = true;
        mechanism = new EeshMechanism(hardwareMap);
        limitSwitch = hardwareMap.touchSensor.get("limitSwitch");
        mechanism.setWrist(EeshMechanism.WRISTHOVER);
        frontLeft = hardwareMap.dcMotor.get("leftFront");
        frontRight = hardwareMap.dcMotor.get("leftRear");
        backRight = hardwareMap.dcMotor.get("rightRear");
    }

    @Override
    public void loop() {
        mechanism.update();
        if(Math.abs(gamepad2.left_stick_y)>0.25) {
            mechanism.setWorm(-gamepad2.left_stick_y * pow);
        }
        if(gamepad2.dpad_up)
        {
            mechanism.setWorm(0.2);
        }



        if(limitSwitch.isPressed())
        {
            pow = 0;
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            requestOpModeStop();
        }

        telemetry.addData("limit switch pressed", limitSwitch.isPressed());
        telemetry.addData("worm pos", mechanism.wormCurrent);

        telemetry.update();

    }
}
