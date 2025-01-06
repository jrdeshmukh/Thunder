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
        Worm.overrideLimit = false;
        mechanism = new EeshMechanism(hardwareMap);
        mechanism.setWrist(EeshMechanism.WRISTHOVER);
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
        mechanism.update();
        dashboardPoseTracker.update();

        //red = colorRangeSensor.red();
        //blue = colorRangeSensor.blue();
        //green = colorRangeSensor.green();
        //updateState();
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

        if(gp2.dpad_right()) {
            mechanism.slide.runToPos(500);
        }

        autoDriving = follower.isBusy()|| autoDriving;

        runningActions = newActions;
        //dash.sendTelemetryPacket(packet);

        if((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) != 0 || !autoDriving) {
            if(autoDriving) {
                autoDriving = false;
                follower.startTeleopDrive();
            }
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y*speedMod*sprint, -gamepad1.left_stick_x*speedMod*sprint*1.27, -gamepad1.right_stick_x*speedMod*0.75*sprint);
        }


        if(Math.abs(gamepad2.right_stick_y)>0) {
            mechanism.slide.setPower(-gamepad2.right_stick_y);
            autoSlide = false;
        }
        else {
            autoSlide = true;
            mechanism.slide.setPow2();
        }

        if(mechanism.wormCurrent <= 1450 && mechanism.wormCurrent >= 0)
        {
            //mechanism.setWrist(EeshMechanism.WRISTHOVER);
        }


        if(Math.abs(gamepad2.left_stick_y)>0.25) {
            telemetry.addData("Joystick State", 1);

            if(Math.abs(-gamepad2.left_stick_y) > 0.5 && mechanism.wormCurrent > -500)
            {
                mechanism.setWorm(-gamepad2.left_stick_y);
            }
            else if(mechanism.wormCurrent <= -300 && -gamepad2.left_stick_y < 0)
            {
                mechanism.setWorm(-1);
            }
            else if(mechanism.wormCurrent <= -300 && -gamepad2.left_stick_y > 0)
            {
                mechanism.setWorm(-gamepad2.left_stick_y);
            }
            telemetry.addData("worm power mod", -gamepad2.left_stick_y);
        }
        else {
            telemetry.addData("Set Pow 2 State", 1);
            mechanism.worm.setPow2();
        }

        if(gp2.y())                                     mechanism.setChosenAngle(-45);
        if(gp2.a())                                     mechanism.setChosenAngle(EeshMechanism.WRIST_PICKUP_ANGLE);
        if(Math.abs(gamepad2.right_trigger) > 0.1)      mechanism.worm.runToPos((int) mechanism.wormPickup);
        if(Math.abs(gamepad2.left_trigger) > 0.1)       mechanism.setWrist(mechanism.getWristPos() - inc);
        if(gp2.right_bumper())                          mechanism.intake.setPower(1);
        if(gp2.x())                                  mechanism.worm.runToPos((int) mechanism.wormPickup);

        if(gp2.left_bumper())
        {
            mechanism.intake.setPower(-1);
            mechanism.setWrist(0.83);
        }

        if(gamepad2.dpad_down)
        {
            mechanism.setWrist(EeshMechanism.WRISTHOVER);
        }

        if(gamepad2.dpad_up)
        {
            mechanism.setWrist(0.8522);
        }

        if(mechanism.wormCurrent < 0 && !flippedSafety1)
        {
            //mechanism.setWrist(EeshMechanism.WRISTHOVER);
            flippedSafety1 = true;
        }

        if(mechanism.wormCurrent > 0 && flippedSafety1)
        {
            flippedSafety1 = false;
        }

        if(mechanism.wormCurrent < -900 /*&& !flippedSafety2*/)
        {
            //mechanism.setWrist(0.32);
            flippedSafety2 = true;
        }

        if(mechanism.wormCurrent > -900 && flippedSafety2)
        {
            flippedSafety2 = false;
        }


        telemetry.addData("x: ", follower.getPose().getX());
        telemetry.addData("y: ", follower.getPose().getY());
        telemetry.addData("heading: ", follower.getPose().getHeading());
        telemetry.addData("gp1ly",gamepad1.left_stick_y);
        telemetry.addData("worm pos", mechanism.wormCurrent);
        telemetry.addData("copycat pos", mechanism.copycatCurrent);
        telemetry.addData("worm target pos", mechanism.wormCurrent);
        telemetry.addData("worm override limit?", Worm.overrideLimit);
        telemetry.addData("slide pos", mechanism.slideCurrent);
        telemetry.addData("wrist pos", mechanism.getWristPos());
        telemetry.addData("worm power", mechanism.worm.wormPow);
        telemetry.addData("copycat power", mechanism.worm.copycatPow);
        telemetry.addData("wirst angle: ", mechanism.getWristAngle());
        telemetry.addData("speedMod", speedMod);
        telemetry.addData("slide power: ", mechanism.slide.lastPower);
        telemetry.addData("worm pickup pos: ", mechanism.wormPickup);
        telemetry.addData("auto slide: ", autoSlide);
        //telemetry.addData("worm current: ",  mechanism.worm.worm.getCurrent(CurrentUnit.MILLIAMPS));
        //telemetry.addData("copyact current: ", mechanism.worm.copycat.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("isbudy", follower.isBusy());
        telemetry.addData("autodriving", autoDriving);
        telemetry.addData("wprm angle: ", mechanism.worm.getAngle());
        telemetry.addData("loop time: ", looptime);
        telemetry.addData("Red: ", red);
        telemetry.addData("Blue: ", blue);
        telemetry.addData("Green: ", green);
        //telemetry.addData("Distance: ", colorRangeSensor.getDistance(DistanceUnit.INCH));
        //telemetry.addData("Dont Break Intake Pressed?", mechanism.worm.dontBreakIntake.isPressed());
        //telemetry.addData("Dont Break Motor Pressed?", mechanism.worm.dontBreakMotor.isPressed());
        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(follower.getPose(), "#4CAF50");
        Drawing.sendPacket();

        telemetry.update();

    }

    public void updateState() {
        switch (state) {
            case INTAKING:
                if(firstStateSwitch) {
                    mechanism.intake.setPower(1);
                    firstStateSwitch = false;
                }
                mechanism.setChosenAngle(EeshMechanism.WRIST_PICKUP_ANGLE);
                if(colorRangeSensor.getDistance(DistanceUnit.INCH) < EeshMechanism.INTAKE_DISTANCE) {
                    state = IntakeState.HOLDING;
                    firstStateSwitch = true;
                }

            case HOLDING:
                if(firstStateSwitch) {
                    mechanism.intake.setPower(1);
                    firstStateSwitch = false;
                }
                //red
                if(red > 200 && blue < 100 && green < 150) {
                    if(!redTeam) {
                        state = IntakeState.DROPPING;
                        firstStateSwitch = true;
                    }
                }
                //blue
                if(red < 100 && blue > 200 && green > 150) {
                    if(redTeam) {
                        state = IntakeState.DROPPING;
                        firstStateSwitch = true;
                    }
                }
            case DROPPING:
                if(firstStateSwitch) {
                    mechanism.intake.setPower(-1);
                    firstStateSwitch = false;
                }
                if(colorRangeSensor.getDistance(DistanceUnit.INCH) > EeshMechanism.INTAKE_DISTANCE) {
                    state = IntakeState.INTAKING;
                    firstStateSwitch = true;
                }
            case WAITING:
                if(firstStateSwitch) {
                    mechanism.intake.setPower(0);
                    firstStateSwitch = false;
                }
        }

    }
}
