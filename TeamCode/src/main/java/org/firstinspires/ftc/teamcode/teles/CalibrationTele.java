package org.firstinspires.ftc.teamcode.teles;

import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.wrappers.BBG;
import org.firstinspires.ftc.teamcode.wrappers.EeshMechanism;
import org.firstinspires.ftc.teamcode.wrappers.Slide;
import org.firstinspires.ftc.teamcode.wrappers.Worm;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@TeleOp()
public class CalibrationTele extends OpMode {
    Follower follower;
    Worm worm;
    Slide slide;
    EeshMechanism mechanism;
    BBG gp1, gp2;
    int target = 0;

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        mechanism = new EeshMechanism(hardwareMap);

        gp1 = new BBG(gamepad1);
        gp2 = new BBG(gamepad2);

        //claw2 = new CalibrateServo(claw.claw);
        //wrist2 = new CalibrateServo(wrist.wrist);
    }

    @Override
    public void loop() {
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        mechanism.update();

        double wristPos = mechanism.getWristPos();
        double increment = 0.01;

        if(gp2.dpad_down()) {
            mechanism.setWrist(wristPos - increment);
        }
        if(gp2.dpad_up()) {
            mechanism.setWrist(wristPos + increment);
        }



        mechanism.slide.setPower(-gamepad2.right_stick_y);
        mechanism.worm.setPower(-gamepad2.left_stick_y);




        telemetry.addData("Wrist Pos: ", mechanism.wristL.getPosition());
        telemetry.addData("Slide Pos", mechanism.slideCurrent);
        telemetry.addData("Wrist Angle: ", mechanism.getWristAngle());
        telemetry.addData("Slide power: ", gamepad2.right_stick_y);

        telemetry.addData("left stick x", gamepad1.left_stick_x);
        telemetry.addData("left stick y", gamepad1.left_stick_y);
        telemetry.addData("right stick x", gamepad1.right_stick_x);
        telemetry.update();
    }
}
