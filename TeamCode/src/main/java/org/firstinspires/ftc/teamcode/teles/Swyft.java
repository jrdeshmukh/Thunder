package org.firstinspires.ftc.teamcode.teles;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.wrappers.Worm;

@TeleOp
public class Swyft extends OpMode {

    DcMotorEx leftFront, leftRear, rightFront, rightRear, slide;
    Worm worm;
    @Override
    public void init() {

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        slide = hardwareMap.get(DcMotorEx.class, "slide");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);

        worm = new Worm(hardwareMap);


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x; // this is strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator;
        double leftRearPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightRearPower = (y + x - rx) / denominator;

        slide.setPower(-gamepad2.right_stick_y);
        worm.setPower(-gamepad2.left_stick_y);

        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);


        telemetry.addData("slidepos: ", slide.getCurrentPosition());
        telemetry.addData("worm pos: ", worm.worm.getCurrentPosition());
        telemetry.addData("worm power: ", -gamepad2.left_stick_y);
        telemetry.addData("left stick y: ", gamepad2.left_stick_y);
        telemetry.addData("slide power: ", -gamepad2.right_stick_y);
        telemetry.addData("right stick y: ", gamepad2.right_stick_y);
        telemetry.addData("rightFront power: ", rightFrontPower);
        telemetry.addData("leftFront power: ", leftFrontPower);
        telemetry.addData("rightRear power: ", rightRearPower);
        telemetry.addData("leftRear power: ", leftRearPower);

        telemetry.update();
    }
}
