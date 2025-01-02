package org.firstinspires.ftc.teamcode.teles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.wrappers.MecanumDrive;
import org.firstinspires.ftc.teamcode.wrappers.BBG;
import org.firstinspires.ftc.teamcode.wrappers.EeshMechanism;
import org.firstinspires.ftc.teamcode.wrappers.Worm;

import java.util.ArrayList;
import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp
public class AutoTele extends OpMode {
    private EeshMechanism mechanism;
    double inc = 0.015;
    double pow = 1;
    double speedMod = 0.5;
    double sprint = 1;

    public boolean flippedSafety1, flippedSafety2;
    RevBlinkinLedDriver leds;
    FtcDashboard dash = FtcDashboard.getInstance();


    Pose basket = new Pose(-54.1, -54.1, 5*Math.PI/4);
    Follower follower;
    Path basketPath;
    List<Action> runningActions = new ArrayList<>();
    Timer timer;
    boolean autoDriving = false, firstTime = false;
    double looptime = 0.0;






    public BBG gp1, gp2;

    @Override
    public void init() {
        Worm.overrideLimit = false;
        mechanism = new EeshMechanism(hardwareMap);
        mechanism.setWrist(EeshMechanism.WRISTHOVER);
        //leds = hardwareMap.get(RevBlinkinLedDriver.class, "leds");

        gp1 = new BBG(gamepad1);
        gp2 = new BBG(gamepad2);

        flippedSafety1 = false;
        flippedSafety2 = false;
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        basketPath = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(40,0, Point.CARTESIAN)));
        follower.followPath(basketPath);
    }

    @Override
    public void loop() {
        if (!firstTime) {
            timer = new Timer();
            firstTime = true;
        }
        else {
            looptime = timer.getElapsedTime();
            firstTime = false;
        }
        follower.update();
        mechanism.update();
        //leds.setPattern(new Patter);

        if(gp1.dpad_up()) speedMod += 0.1;
        if(gp1.dpad_down()) speedMod -= 0.1;

        sprint = Math.abs(gamepad1.right_trigger) > 0.25 ? 2:1;

        /*if(gp2.dpad_up()) {
            runningActions.add(new SequentialAction(

            ));
        }
        if(gp2.dpad_down()) {
            runningActions.add(new SequentialAction(

            ));
        }*/


        if(gp1.a()) {
            basket = follower.getPose();
        }

        if (gp1.b()) {
            Pose curPose = follower.getPose();
            //basketPath = new Path(
              //      new BezierLine(
                //            new Point(curPose),
                  //          new Point(basket)
                    //)
           // );
            basketPath.setLinearHeadingInterpolation(curPose.getHeading(), basket.getHeading());
            //this.basketPath = new PathChain(basmet);
            follower.followPath(basketPath);
        }

        if(gp2.dpad_left() || gp1.dpad_left()) {
            runningActions = new ArrayList<>();
            //mechanism.worm.runToPos(mechanism.worm.worm.getCurrentPosition());
            //mechanism.slide.runToPos(mechanism.slide.slide.getCurrentPosition());
            mechanism.setWorm(0);
            mechanism.slide.setPower(0);
        }

        TelemetryPacket packet = new TelemetryPacket();

        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }

        autoDriving = follower.isBusy()|| autoDriving;

        runningActions = newActions;
        //dash.sendTelemetryPacket(packet);

        if((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) != 0 || !autoDriving) {
            if(autoDriving) {
                autoDriving = false;
                follower.startTeleopDrive();
            }
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y*speedMod*sprint, -gamepad1.left_stick_x*speedMod*sprint, -gamepad1.right_stick_x*speedMod*0.75*sprint);
        }


        if(Math.abs(gamepad2.right_stick_y)>0) {
            mechanism.slide.runToPos(EeshMechanism.slideCurrent);
            mechanism.slide.setPower(-gamepad2.right_stick_y);
            telemetry.addData("slide power mod", -gamepad2.right_stick_y);

        }

        if(EeshMechanism.wormCurrent <= 1450 && EeshMechanism.wormCurrent >= 0)
        {
            mechanism.setWrist(EeshMechanism.WRISTHOVER);
        }


        if(Math.abs(gamepad2.left_stick_y)>0.25) {
            if(Math.abs(-gamepad2.left_stick_y) > 0.5 && EeshMechanism.wormCurrent > -500)
            {
                mechanism.setWorm(-gamepad2.left_stick_y);
            }
            else if(EeshMechanism.wormCurrent <= -300 && -gamepad2.left_stick_y < 0)
            {
                mechanism.setWorm(-1);
            }
            else if(EeshMechanism.wormCurrent <= -300 && -gamepad2.left_stick_y > 0)
            {
                mechanism.setWorm(-gamepad2.left_stick_y);
            }
            telemetry.addData("worm power mod", -gamepad2.left_stick_y);
        }
        else mechanism.setWorm(0);

        if(gp2.y())                                     mechanism.setWrist(EeshMechanism.WRISTHOVER);
        if(gp2.a())                                     mechanism.setWrist(EeshMechanism.WRISTDOWNPOSSUBMERSIBLE);
        if(Math.abs(gamepad2.right_trigger) > 0.1)      mechanism.setWrist(mechanism.getWristPos() + inc);
        if(Math.abs(gamepad2.left_trigger) > 0.1)       mechanism.setWrist(mechanism.getWristPos() - inc);
        if(gp2.right_bumper())                          mechanism.intake.setPower(1);

        if(gp2.left_bumper())
        {
            mechanism.intake.setPower(-1);
            mechanism.setWrist(0.83);
        }

        if(gamepad2.dpad_down)
        {
            mechanism.worm.worm.setTargetPosition(-1115);
            mechanism.worm.setPow3();
            mechanism.setWrist(EeshMechanism.WRISTHOVER);
        }

        if(gamepad2.dpad_up)
        {
            mechanism.worm.worm.setTargetPosition(1546);
            mechanism.worm.setPow3();
            mechanism.setWrist(0.8522);
        }

        if(EeshMechanism.wormCurrent < 0 && !flippedSafety1)
        {
            mechanism.setWrist(EeshMechanism.WRISTHOVER);
            flippedSafety1 = true;
        }

        if(EeshMechanism.wormCurrent > 0 && flippedSafety1)
        {
            flippedSafety1 = false;
        }

        if(EeshMechanism.wormCurrent < -900 /*&& !flippedSafety2*/)
        {
            mechanism.setWrist(0.32);
            mechanism.setWorm(0.4);
            flippedSafety2 = true;
        }

        if(EeshMechanism.wormCurrent > -900 && flippedSafety2)
        {
            flippedSafety2 = false;
        }

        if(EeshMechanism.wormCurrent > 1500 /*&& !flippedSafety2*/)
        {
            mechanism.setWorm(-0.4);
        }

        telemetry.addData("worm pos", EeshMechanism.wormCurrent);
        telemetry.addData("worm target pos", EeshMechanism.wormCurrent);
        telemetry.addData("worm override limit?", Worm.overrideLimit);
        telemetry.addData("slide pos", EeshMechanism.slideCurrent);
        telemetry.addData("wrist pos", mechanism.getWristPos());
        telemetry.addData("speedMod", speedMod);
        telemetry.addData("isbudy", follower.isBusy());
        telemetry.addData("autodriving", autoDriving);
        telemetry.addData("loop time", looptime);

        telemetry.addData("Dont Break Intake Pressed?", mechanism.worm.dontBreakIntake.isPressed());
        telemetry.addData("Dont Break Motor Pressed?", mechanism.worm.dontBreakMotor.isPressed());


        telemetry.update();

    }
}
