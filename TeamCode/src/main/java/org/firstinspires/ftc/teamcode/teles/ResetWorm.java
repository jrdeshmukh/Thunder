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



    @Override
    public void init() {
        Worm.overrideLimit = true;
        mechanism = new EeshMechanism(hardwareMap);
        limitSwitch = hardwareMap.touchSensor.get("limitSwitch");
        mechanism.setWrist(EeshMechanism.WRISTHOVER);
    }

    @Override
    public void loop() {
        if(Math.abs(gamepad2.left_stick_y)>0.25) {
            mechanism.worm.runToPos(mechanism.worm.worm.getCurrentPosition());
            mechanism.setWorm(-gamepad2.left_stick_y * pow);
        }


        if(limitSwitch.isPressed())
        {
            pow = 0;
            mechanism.worm.worm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        telemetry.addData("limit switch pressed", limitSwitch.isPressed());
        telemetry.addData("worm pos", mechanism.worm.worm.getCurrentPosition());

        telemetry.update();

    }
}
