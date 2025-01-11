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
    double lastPow = 0;




    @Override
    public void init() {
        Worm.overrideLimit = true;
        mechanism = new EeshMechanism(hardwareMap);
        limitSwitch = hardwareMap.touchSensor.get("limitSwitch");
        mechanism.setWrist(EeshMechanism.WRISTHOVER);
        frontLeft = hardwareMap.dcMotor.get("leftFront");
        frontRight = hardwareMap.dcMotor.get("leftRear");
        backRight = hardwareMap.dcMotor.get("rightRear");
        mechanism.setChosenAngle(180);
    }

    @Override
    public void loop() {
        mechanism.update();
        if(Math.abs(gamepad2.left_stick_y)>0.25) {
            mechanism.worm.worm.setPower(-gamepad2.left_stick_y * pow);
            mechanism.worm.copycat.setPower(-gamepad2.left_stick_y * pow);
            lastPow = -gamepad2.left_stick_y * pow;
        }




        if(limitSwitch.isPressed() && lastPow>0)
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
