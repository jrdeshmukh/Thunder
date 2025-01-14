package org.firstinspires.ftc.teamcode.teles;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.wrappers.BBG;
import org.firstinspires.ftc.teamcode.wrappers.EeshMechanism;
import org.firstinspires.ftc.teamcode.wrappers.Worm;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp
public class JayAutoTele extends OpMode {
    private EeshMechanism mechanism;
    double inc = 0.015;
    double pow = 1;
    double speedMod = 0.5;
    double sprint = 1;

    public boolean flippedSafety1, flippedSafety2;
    RevBlinkinLedDriver leds;
    FtcDashboard dash = FtcDashboard.getInstance();


    Pose basket = new Pose();
    Pose subPose = new Pose();
    Pose cp = new Pose();
    Follower follower;
    PathChain basketPath;
    List<Action> runningActions = new ArrayList<>();
    ColorRangeSensor colorRangeSensor;
    DashboardPoseTracker dashboardPoseTracker;
    Servo sweeper;
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


    public class WaitUntilDone implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return follower.isBusy();
        }
    }

    public Action followUntilDone(PathChain a) {
        return new FollowUntilDone(a);
    }

    public class FollowUntilDone implements Action {

        PathChain a;
        boolean called = false;

        public FollowUntilDone(PathChain a)  {
            this.a = a;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!called) {
                follower.followPath(a, true);
                called = true;
            }
            return follower.isBusy();
        }
    }

    public Action follow(PathChain a) {
        return new InstantAction(() -> {
            if (!follower.isBusy()) {
                follower.followPath(a, true);
            }
        });
    }



    public BBG gp1, gp2;

    @Override
    public void init() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        Worm.overrideLimit = false;
        mechanism = new EeshMechanism(hardwareMap);
        //mechanism.setWrist(EeshMechanism.WRISTHOVER);
        //colorRangeSensor = hardwareMap.get(ColorRangeSensor.class, "crs");
        //leds = hardwareMap.get(RevBlinkinLedDriver.class, "leds");

        gp1 = new BBG(gamepad1);
        gp2 = new BBG(gamepad2);

        sweeper = hardwareMap.servo.get("sweeper");


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


        sprint = Math.abs(gamepad1.right_trigger) > 0.25 ? 2:1;
        if(Math.abs(gamepad1.left_trigger)>0.1) {
            sprint = 0.4;
        }





        if(gp1.dpad_up()) {
            runningActions.add(new SequentialAction(
                    mechanism.worm.autoMove(1420),
                    mechanism.worm.waitUntilDone(),
                    mechanism.slide.autoMove(2960),
                    mechanism.slide.waitUntilDone(),
                    new InstantAction(() -> mechanism.setChosenAngle(220))
            ));
        }


        if(gp1.dpad_right()) sweeper.setPosition(0);
        if(gp1.dpad_left()) sweeper.setPosition(0.85);


        if(gp1.a()) {
            basket = follower.getPose();
            if (Objects.equals(cp, new Pose())) {
                cp = basket;
            }
        }

        if(gp1.x()) {
            cp = follower.getPose();
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
            if(!follower.isBusy()) runningActions.add(new SequentialAction(
                    new InstantAction(() -> mechanism.setChosenAngle(15)),
                    mechanism.slide.liftBottom(),
                    mechanism.slide.waitUntilDone(),
                    mechanism.worm.autoMove(1420),
                    follow(basketPath),
                    mechanism.worm.waitUntilDone(),
                    mechanism.slide.autoMove(2960),
                    mechanism.slide.waitUntilDone(),
                    new InstantAction(() -> mechanism.setChosenAngle(220))
            ));
        }
        if (gp1.left_bumper()) {
            Pose curPose = follower.getPose();
            subPose = curPose;
            basketPath = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Point(curPose),
                                    new Point(cp),
                                    new Point(basket)
                            )
                    )
                    .setLinearHeadingInterpolation(curPose.getHeading(), basket.getHeading())
                    .build();
        if(!follower.isBusy()) runningActions.add(new SequentialAction(
                    new InstantAction(() -> mechanism.setChosenAngle(15)),
                    mechanism.slide.liftBottom(),
                    mechanism.slide.waitUntilDone(),
                    mechanism.worm.autoMove(1420),
                    follow(basketPath),
                    mechanism.worm.waitUntilDone(),
                mechanism.slide.autoMove(2960),
                mechanism.slide.waitUntilDone(),
                new InstantAction(() -> mechanism.setChosenAngle(220))
                ));
        }


        if(gp2.dpad_down()) {
            EeshMechanism.ROHAN_WORM_POS -= 10;
            mechanism.worm.runToPos(EeshMechanism.ROHAN_WORM_POS);
        }
        if(gp2.dpad_up()) {
            EeshMechanism.ROHAN_WORM_POS += 10;
            mechanism.worm.runToPos(EeshMechanism.ROHAN_WORM_POS);
        }

        if(gp1.dpad_down()) {
            runningActions.add(new SequentialAction(
                        mechanism.setAngleAction(15),
                        new SleepAction(0.5),
                        mechanism.slide.liftBottom(),
                        mechanism.slide.waitUntilDone(),
                        mechanism.startIntake(),
                        mechanism.worm.autoMove(EeshMechanism.ROHAN_WORM_POS)
                ));
            }

        if(gp1.right_bumper()){
                Pose curPose = follower.getPose();
                PathChain subPath = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Point(curPose),
                                        new Point(cp),
                                        new Point(subPose)
                                )
                        )
                        .setLinearHeadingInterpolation(curPose.getHeading(), subPose.getHeading())
                        .build();
                runningActions.add(new SequentialAction(
                        mechanism.setAngleAction(15),
                        new SleepAction(0.5),
                        mechanism.slide.liftBottom(),
                        mechanism.slide.waitUntilDone(),
                        mechanism.startIntake(),
                        mechanism.worm.autoMove(EeshMechanism.ROHAN_WORM_POS),
                        follow(subPath))
                );
            }


        if(gp2.y()) {
            mechanism.worm.runToPos(EeshMechanism.ROHAN_WORM_POS);
            mechanism.slide.runToPos(EeshMechanism.ROHAN_SLIDE_POS);
        }




        if(gp2.left_bumper())
        {
            runningActions.add(new SequentialAction(
                    mechanism.drop(),
                    new SleepAction(2),
                    mechanism.startIntake()
            ));
        }
        

        if(gp2.ps || gp1.ps || gp1.right_stick_button || gp1.left_stick_button) {
            runningActions = new ArrayList<>();
            mechanism.setWorm(0);
            mechanism.slide.setPower(0);
            follower.breakFollowing();
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


        if(gamepad2.right_stick_y!=0) {
            mechanism.slide.setPower(-gamepad2.right_stick_y);
            autoSlide = false;
        }
        else {
            autoSlide = true;
            mechanism.slide.setPow2();
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
        if(gp2.b())                                     mechanism.setChosenAngle(0);


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
