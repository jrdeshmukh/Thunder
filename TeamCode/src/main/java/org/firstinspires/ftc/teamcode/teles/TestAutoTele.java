package org.firstinspires.ftc.teamcode.teles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.wrappers.BBG;
import org.firstinspires.ftc.teamcode.wrappers.EeshMechanism;
import org.firstinspires.ftc.teamcode.wrappers.Worm;

import java.util.ArrayList;
import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp
public class TestAutoTele extends OpMode {
    double inc = 0.015;
    double pow = 1;
    double speedMod = 0.5;
    double sprint = 1;

    public boolean flippedSafety1, flippedSafety2;
    RevBlinkinLedDriver leds;
    FtcDashboard dash = FtcDashboard.getInstance();


    Pose basket = new Pose(-54.1, -54.1, 5*Math.PI/4);
    Follower follower;
    PathChain basketPath;
    List<Action> runningActions = new ArrayList<>();
    ColorRangeSensor colorRangeSensor;
    DashboardPoseTracker dashboardPoseTracker;
    Timer timer;
    boolean autoDriving = false, firstTime = false, autoSlide = true;
    double looptime = 0.0;
    boolean firstStateSwitch= true;

    enum IntakeState {
        INTAKING,
        HOLDING,
        WRONG_INTAKE,
        DROPPING,
        WAITING,
    }
    IntakeState state = IntakeState.WAITING;

    boolean redTeam = true;
    int red, blue, green;





    public BBG gp1, gp2;

    @Override
    public void init() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //colorRangeSensor = hardwareMap.get(ColorRangeSensor.class, "crs");
        //leds = hardwareMap.get(RevBlinkinLedDriver.class, "leds");

        gp1 = new BBG(gamepad1);
        gp2 = new BBG(gamepad2);


        flippedSafety1 = false;
        flippedSafety2 = false;

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.startTeleopDrive();

        dashboardPoseTracker = new DashboardPoseTracker(follower.poseUpdater);
        Drawing.drawRobot(follower.getPose(), "#4CAF50");
        Drawing.sendPacket();
        timer = new Timer();
    }

    @Override
    public void loop() {
        looptime = timer.getElapsedTime();
        timer = new Timer();
        follower.update();
        dashboardPoseTracker.update();


        if(gp1.dpad_up()) speedMod += 0.1;
        if(gp1.dpad_down()) speedMod -= 0.1;

        sprint = Math.abs(gamepad1.right_trigger) > 0.25 ? 2:1;


        if(gp1.a()) {
            basket = follower.getPose();
        }

        if (gp1.b()) {
            Pose curPose = follower.getPose();
            basketPath = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Point(curPose),
                                    new Point(basket)
                            )
                    )
                    .setLinearHeadingInterpolation(curPose.getHeading(), basket.getHeading())
                    .build();
            if(!follower.isBusy()) follower.followPath(basketPath, true);
        }
        follower.update();




        autoDriving = follower.isBusy()|| autoDriving;


        if((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) != 0 || !autoDriving) {
            if(autoDriving) {
                autoDriving = false;
                follower.startTeleopDrive();
            }
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y*speedMod*sprint, -gamepad1.left_stick_x*speedMod*sprint*1.27, -gamepad1.right_stick_x*speedMod*0.75*sprint);
        }





        telemetry.addData("x: ", follower.getPose().getX());
        telemetry.addData("y: ", follower.getPose().getY());
        telemetry.addData("heading: ", follower.getPose().getHeading());
        telemetry.addData("speedMod", speedMod);
        telemetry.addData("isbudy", follower.isBusy());
        telemetry.addData("autodriving", autoDriving);
        telemetry.addData("loop time: ", looptime);
        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(follower.getPose(), "#4CAF50");
        Drawing.sendPacket();
        telemetry.update();

    }

}
