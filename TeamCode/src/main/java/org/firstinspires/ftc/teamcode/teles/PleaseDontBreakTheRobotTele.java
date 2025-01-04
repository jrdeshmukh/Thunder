package org.firstinspires.ftc.teamcode.teles;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.wrappers.MecanumDrive;
import org.firstinspires.ftc.teamcode.wrappers.BBG;
import org.firstinspires.ftc.teamcode.wrappers.EeshMechanism;
import org.firstinspires.ftc.teamcode.wrappers.Worm;

@TeleOp
public class PleaseDontBreakTheRobotTele extends OpMode {
    private EeshMechanism mechanism;
    double inc = 0.015;
    double deadzone = 0.25;
    double pow = 1;
    double speedMod = 0.5;
    boolean sprint = false;

    public MecanumDrive drive;
    public boolean flippedSafety1, flippedSafety2;


    public BBG gp1, gp2;
    public boolean spinning = false;

    @Override
    public void init() {
        Worm.overrideLimit = false;
        mechanism = new EeshMechanism(hardwareMap);
        mechanism.setWrist(EeshMechanism.WRISTHOVER);
        drive = new MecanumDrive(hardwareMap);

        gp1 = new BBG(gamepad1);
        gp2 = new BBG(gamepad2);

        flippedSafety1 = false;
        flippedSafety2 = false;
    }

    @Override
    public void loop() {
        if(gp1.dpad_up()) speedMod += 0.1;
        if(gp1.dpad_down()) speedMod -= 0.1;

        sprint = Math.abs(gamepad1.right_trigger) > 0.25;


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

        drive.leftFront.setPower(sprint ? leftFrontPower * speedMod * 2 : leftFrontPower * speedMod);
        drive.leftBack.setPower(sprint ? leftRearPower * speedMod * 2: leftRearPower * speedMod);
        drive.rightFront.setPower(sprint ? rightFrontPower * speedMod * 2:  rightFrontPower * speedMod);
        drive.rightBack.setPower(sprint ? rightRearPower * speedMod * 2: rightRearPower * speedMod);

        /*if(mechanism.worm.worm.getCurrentPosition() <= 0 && mechanism.slide.slide.getCurrentPosition() >= 20)
        {
            mechanism.slide.setPower(-1);
            mechanism.setWrist(EeshMechanism.WRISTHOVERHIGH);
        }
        else
        if(mechanism.worm.worm.getCurrentPosition() >= 0 && mechanism.slide.slide.getCurrentPosition() <= 2400)
        {
            mechanism.slide.setPower(((double) (2400 - mechanism.slide.slide.getCurrentPosition() / 2400)));
        }
        if(mechanism.worm.worm.getCurrentPosition() <= 0 && mechanism.slide.slide.getCurrentPosition() <= 20)
        {
            mechanism.slide.setPower(0);
            mechanism.setWrist(EeshMechanism.WRISTHOVERHIGH);
        }*/
        if(Math.abs(gamepad2.right_stick_y)>0) {
            mechanism.slide.runToPos(mechanism.slide.slide.getCurrentPosition());
            mechanism.slide.setPower(-gamepad2.right_stick_y);
            telemetry.addData("slide power mod", -gamepad2.right_stick_y);

        }

        if(mechanism.worm.worm.getCurrentPosition() <= 1450 && mechanism.worm.worm.getCurrentPosition() >= 0)
        {
            mechanism.setWrist(EeshMechanism.WRISTHOVER);
        }
        /*if(mechanism.worm.worm.getCurrentPosition() <= 1350 && mechanism.slide.slide.getCurrentPosition() >= 0)
        {
            mechanism.slide.setPower(-1);
        }
        else if(mechanism.worm.worm.getCurrentPosition() <= 0){
            mechanism.slide.setPower(0);
        }*/

        if(Math.abs(gamepad2.left_stick_y)>0.25) {
            double pMod = -gamepad2.left_stick_y * pow;
           // mechanism.worm.runToPos((int) (mechanism.worm.worm.getCurrentPosition() + Math.signum(pMod)));
            if(Math.abs(pMod) > 0.5 && mechanism.worm.worm.getCurrentPosition() > -500)
            {
                mechanism.setWorm(pMod);
            }
            else if(mechanism.worm.worm.getCurrentPosition() <= -300 && pMod < 0)
            {
                mechanism.setWorm(-1);
            }
            else if(mechanism.worm.worm.getCurrentPosition() <= -300 && pMod > 0)
            {
                mechanism.setWorm(pMod);
            }
            //mechanism.setWorm(Math.abs(pMod) > 0.5 ? pMod : 0);
            telemetry.addData("worm power mod", -gamepad2.left_stick_y * pow);
        }
        else mechanism.setWorm(0);

        if(gp2.y())
        {
            mechanism.setWrist(EeshMechanism.WRISTHOVER);
        }

        if(gp2.a())
        {
            mechanism.setWrist(EeshMechanism.WRISTDOWNPOSSUBMERSIBLE);
        }

        if(Math.abs(gamepad2.right_trigger) > 0.1)
        {
            mechanism.setWrist(mechanism.wristL.getPosition() + inc);
        }

        if(Math.abs(gamepad2.left_trigger) > 0.1)
        {
            mechanism.setWrist(mechanism.wristL.getPosition() - inc);
        }

        if(gp2.right_bumper())
        {
            mechanism.intake.setPower(1);
            /*if(spinning)
            {
                mechanism.intake.setPower(0);
                spinning = false;
            }
            else
            {
                mechanism.intake.setPower(1);
                spinning = true;
            }*/
        }

        if(gp2.left_bumper())
        {
            mechanism.intake.setPower(-1);
            mechanism.setWrist(0.83);

            /*if(spinning)
            {
                mechanism.intake.setPower(0);
                spinning = false;
            }
            else
            {
                mechanism.intake.setPower(-0.5);
                mechanism.setWrist(0.83);
                spinning = true;
            }*/
        }

        if(gamepad2.dpad_down)
        {
            mechanism.worm.worm.setTargetPosition(-1115);
            mechanism.setWrist(EeshMechanism.WRISTHOVER);
        }

        if(gamepad2.dpad_up)
        {
            mechanism.worm.worm.setTargetPosition(1546);
            mechanism.setWrist(0.8522);
        }

        if(mechanism.worm.worm.getCurrentPosition() < 0 && !flippedSafety1)
        {
            mechanism.setWrist(EeshMechanism.WRISTHOVER);
            flippedSafety1 = true;
        }

        if(mechanism.worm.worm.getCurrentPosition() > 0 && flippedSafety1)
        {
            flippedSafety1 = false;
        }

        if(mechanism.worm.worm.getCurrentPosition() < -900 /*&& !flippedSafety2*/)
        {
            mechanism.setWrist(0.32);
            mechanism.setWorm(0.4);
            flippedSafety2 = true;
        }

        if(mechanism.worm.worm.getCurrentPosition() > -900 && flippedSafety2)
        {
            flippedSafety2 = false;
        }

        if(mechanism.worm.worm.getCurrentPosition() > 1500 /*&& !flippedSafety2*/)
        {
            mechanism.setWorm(-0.4);
        }


        //if(mechanism.worm.dontBreakIntake.isPressed()) mechanism.setWrist(EeshMechanism.WRISTPICKUPLOW);


        telemetry.addData("worm pos", mechanism.worm.worm.getCurrentPosition());
        telemetry.addData("worm target pos", mechanism.worm.worm.getTargetPosition());
        telemetry.addData("worm override limit?", Worm.overrideLimit);
        telemetry.addData("slide pos", mechanism.slide.slide.getCurrentPosition());
        telemetry.addData("wrist pos", mechanism.wristL.getPosition());
        telemetry.addData("speedMod", speedMod);

        telemetry.addData("Dont Break Intake Pressed?", mechanism.worm.dontBreakIntake.isPressed());
        telemetry.addData("Dont Break Motor Pressed?", mechanism.worm.dontBreakMotor.isPressed());


        telemetry.update();

    }
}
