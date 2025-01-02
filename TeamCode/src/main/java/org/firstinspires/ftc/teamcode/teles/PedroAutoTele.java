package org.firstinspires.ftc.teamcode.teles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.wrappers.BBG;
import org.firstinspires.ftc.teamcode.wrappers.EeshMechanism;
import org.firstinspires.ftc.teamcode.wrappers.Mechanism;
import org.firstinspires.ftc.teamcode.wrappers.Slide;
import org.firstinspires.ftc.teamcode.wrappers.Worm;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp()
public class PedroAutoTele extends OpMode {
    BBG gp1, gp2;
    Telemetry telemetryA;

    Pose basket = new Pose(-54.1, -54.1, 5*Math.PI/4);
    FtcDashboard dash = FtcDashboard.getInstance();
    Follower follower;
    Mechanism mechanism;
    Path basketPath;
    boolean autoDriving = false;
    boolean pickup = false;
    boolean autoWorm = true, autoSlide = true;
    boolean holdPoint = false;




    double speedMod = 0.75, wristTarget = 0;
    ElapsedTime start;
    List<Action> runningActions = new ArrayList<>();

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        basketPath = new Path(new BezierPoint(new Point(follower.getPose().getX(), follower.getPose().getY())));
        follower.followPath(basketPath);


        gp2 = new BBG(gamepad2);
        gp1 = new BBG(gamepad1);
        start = new ElapsedTime();
        mechanism = new Mechanism(hardwareMap);
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("idk");
        telemetryA.update();
    }

    @Override
    public void loop() {
        follower.update();
        mechanism.update();

        TelemetryPacket packet = new TelemetryPacket();




        if(gp2.dpad_up()) {
            runningActions.add(new SequentialAction(
                    new InstantAction(() -> mechanism.setChosenAngle(Mechanism.WRIST_OUT)),
                    new InstantAction(() -> mechanism.setClaw(Mechanism.CLAW_CLOSE)),
                    new InstantAction(() -> pickup = false),
                    mechanism.worm.autoMove(2827),
                    mechanism.worm.waitUntilDone(),
                    mechanism.slide.autoMove(2500),
                    mechanism.slide.waitUntilDone(),
                    new InstantAction(() -> mechanism.setChosenAngle(Mechanism.WRIST_SCORE))
            ));
        }
        if(gp2.dpad_down()) {
            runningActions.add(new SequentialAction(
                    new InstantAction(() -> mechanism.setClaw(Mechanism.CLAW_OPEN)),
                    new SleepAction(0.5),
                    new InstantAction(() -> mechanism.setChosenAngle(Mechanism.WRIST_OUT)),
                    new SleepAction(0.5),
                    mechanism.slide.liftBottom(),
                    mechanism.worm.autoMove(700)
            ));
        }


        if(gp2.a()) {
            runningActions.add(new SequentialAction(
                    mechanism.worm.autoMove(462),
                    mechanism.worm.waitUntilDone(),
                    mechanism.slide.autoMove(855),
                    mechanism.slide.waitUntilDone(),
                    new InstantAction(() -> mechanism.setClaw(Mechanism.CLAW_OPEN))
            ));
        }


        if(gp1.a()) {
            basket = follower.getPose();
        }

        if(gp1.y()) {
            holdPoint = !holdPoint;
        }

        if (gp1.b()) {
             Pose curPose = follower.getPose();
             this.basketPath = new Path(
                 new BezierLine(
                            new Point(curPose.getX(), curPose.getY()),
                            new Point(basket.getX(), basket.getY())
                 )
             );
             basketPath.setLinearHeadingInterpolation(curPose.getHeading(), basket.getHeading())
             //.setConstantHeadingInterpolation(basket.getHeading())
             ;
            follower.breakFollowing();
            follower.followPath(basketPath);
        }



        if(gp2.dpad_left() || gp1.dpad_left()) {
            runningActions = new ArrayList<>();
            mechanism.worm.runToPos(mechanism.worm.worm.getCurrentPosition());
            mechanism.slide.runToPos(mechanism.slide.slide.getCurrentPosition());
            mechanism.setWorm(0);
            mechanism.slide.setPower(0);
        }


        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }

        autoDriving = follower.isBusy()|| autoDriving;




        runningActions = newActions;
        dash.sendTelemetryPacket(packet);


        if(gamepad1.right_trigger>0.03) speedMod = 1;
        if(gamepad1.right_bumper) speedMod = 0.75;
        if(gamepad1.left_trigger>0.03) speedMod = 0.25;
        if(gamepad1.left_bumper) speedMod = 0.5;



        if((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) != 0 || !autoDriving) {
            if(autoDriving) {
                autoDriving = false;
                follower.startTeleopDrive();
            }
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y*speedMod, -gamepad1.left_stick_x*speedMod, -gamepad1.right_stick_x*speedMod*0.75);
        }

        if(Math.abs(gamepad2.right_stick_y)>0) {
            mechanism.slide.runToPos(mechanism.slide.slide.getCurrentPosition());
            mechanism.slide.setPower(-gamepad2.right_stick_y);
            autoSlide = false;
        }
        else {
            mechanism.slide.setPow2();
            autoSlide=true;
        }

        if(Math.abs(gamepad2.left_stick_y)>0 && Math.abs(gamepad2.left_stick_x)<0.5) {
            mechanism.worm.runToPos(mechanism.worm.worm.getCurrentPosition());
            mechanism.setWorm(-gamepad2.left_stick_y);
            pickup = false;
            autoWorm = false;
        }
        else {
            mechanism.worm.setPow2();
            autoWorm = true;
        }
        mechanism.setSpin(0.02*-gamepad2.left_stick_x + mechanism.spin.getPosition());




        if (gamepad2.left_bumper) mechanism.setClaw(Mechanism.CLAW_CLOSE);
        if (gamepad2.right_bumper) mechanism.setClaw(Mechanism.CLAW_OPEN);
        if (gamepad2.left_trigger>0.05) mechanism.setChosenAngle(Mechanism.WRIST_DOWN);
        if (gamepad2.right_trigger>0.05) mechanism.setChosenAngle(Mechanism.WRIST_UP);
        if (gamepad2.x)  mechanism.setChosenAngle(Mechanism.WRIST_OUT);
        if (gamepad2.y) mechanism.setChosenAngle(Mechanism.WRIST_SCORE);


        if(pickup){
            mechanism.worm.runToPos((int) mechanism.wormPickup);
        }

        if(gp2.dpad_left())
            mechanism.spin.setPosition(mechanism.spin.getPosition()-0.01);
        if(gp2.dpad_right())
            mechanism.spin.setPosition(mechanism.spin.getPosition()+0.01);






        mechanism.update();

        telemetryA.addData("slide current", mechanism.slide.slide.getCurrentPosition());
        telemetryA.addData("slide max", mechanism.slide.getFullExtension());
        telemetryA.addData("slide target", mechanism.slide.targetPosition);
        telemetryA.addData("worm angle", mechanism.worm.getAngle());
        telemetryA.addData("worm pos", mechanism.worm.worm.getCurrentPosition());
        telemetryA.addData("worm target", mechanism.worm.targetPosition);
        telemetryA.addData("worm pickup", mechanism.wormPickup);
        telemetryA.addData("spin pos", mechanism.spin.getPosition());
        telemetryA.addData("speed mod: ", speedMod);
        telemetryA.addData("x: ", follower.getPose().getX());
        telemetryA.addData("y: ", follower.getPose().getY());
        telemetryA.addData("heading: ", follower.getPose().getHeading());
        telemetryA.addData("running actions: ", runningActions.size());
        telemetryA.addData("autoDriving: ", autoDriving);
        telemetryA.addData("auto worm: ", mechanism.worm.humanControl);
        telemetryA.addData("auto slide: ", mechanism.slide.humanControl);
        //follower.telemetryDebug(telemetryA);
    }
}
