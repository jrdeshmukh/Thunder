/*package org.firstinspires.ftc.teamcode.autos;



import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.teamcode.wrappers.EeshMechanism;
import org.firstinspires.ftc.teamcode.wrappers.Mechanism;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous()
public class FourYellow extends LinearOpMode {

    Follower follower;

    public class FollowerUpdate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            follower.update();
            return follower.isBusy();
        }
    }

    @Override
    public void runOpMode() {
        EeshMechanism mechanism = new EeshMechanism(hardwareMap);
        final Pose startPose = new Pose(9.8, 112.7, 0);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        Path goBasket = new Path(
                new BezierLine(
                        new Point(9.800, 112.700),
                        new Point(20.705, 124.974)
                )
        );
        goBasket.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-40));




        Path turn1 = new Path(
                new BezierPoint(
                        new Point(20.707, 124.976)
                )
        );
        turn1.setConstantHeadingInterpolation(Math.toRadians(-10));

        Path turnBack = new Path(
                new BezierPoint(
                        new Point(20.704, 124.972)
                )
        );
        turnBack.setConstantHeadingInterpolation(Math.toRadians(-45));

        Path turn2 = new Path(
                new BezierPoint(
                        new Point(20.707, 124.976)
                )
        );
        turn2.setConstantHeadingInterpolation(Math.toRadians(11.8));




        waitForStart();
        if(isStopRequested() || mechanism.worm.worm.getCurrentPosition()==0) return;
        Actions.runBlocking(
                new ParallelAction(
                        new FollowerUpdate(),
                        mechanism.worm.setPow(),
                        mechanism.slide.setPow(),
                        mechanism.updateAction(),
                        new SequentialAction(
                                follower.follow(goBasket),
                                //new InstantAction(() -> mechanism.setClaw(Mechanism.CLAW_CLOSE)),
                                //mechanism.worm.autoMove(2950),
                                new SleepAction(1),
                                //mechanism.slide.autoMove(2070),
                                new SleepAction(1.5),
                                //new InstantAction(() -> mechanism.setChosenAngle(Mechanism.WRIST_SCORE)),
                                new SleepAction(0.5),
                                //new InstantAction(() -> mechanism.setClaw(Mechanism.CLAW_OPEN)),
                                new SleepAction(0.5),
                                //new InstantAction(() -> mechanism.setChosenAngle(Mechanism.WRIST_OUT)),
                                new SleepAction(0.5),
                                //mechanism.slide.liftBottom(),
                                new SleepAction(1),
                                //mechanism.worm.autoMove(409),
                                follower.follow(turn1),
                                new SleepAction(0.1),
                                //mechanism.slide.autoMove(1353),
                                new InstantAction(() -> mechanism.setChosenAngle(Mechanism.WRIST_DOWN)),
                                new SleepAction(1),
                                new InstantAction(() -> mechanism.claw.setPosition(Mechanism.CLAW_CLOSE)),
                                new SleepAction(0.5),
                                new InstantAction(() -> mechanism.setChosenAngle(Mechanism.WRIST_UP)),
                                new SleepAction(0.5),
                                mechanism.slide.liftBottom(),
                                new SleepAction(1),
                                mechanism.worm.autoMove(2950),
                                follower.follow(turnBack),
                                new SleepAction(0.5),
                                mechanism.slide.autoMove(2070),
                                new SleepAction(1.5),
                                new InstantAction(() -> mechanism.setChosenAngle(Mechanism.WRIST_SCORE)),
                                new SleepAction(0.5),
                                new InstantAction(() -> mechanism.setClaw(Mechanism.CLAW_OPEN)),
                                new SleepAction(0.5),
                                new InstantAction(() -> mechanism.setChosenAngle(Mechanism.WRIST_OUT)),
                                new SleepAction(0.5),
                                mechanism.slide.liftBottom(),
                                new SleepAction(1),
                                follower.follow(turn2)
                )
        ));

    }

}*/
