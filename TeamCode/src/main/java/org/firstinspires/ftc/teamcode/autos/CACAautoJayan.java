package org.firstinspires.ftc.teamcode.autos;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.wrappers.EeshMechanism;

import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous()
public class CACAautoJayan extends LinearOpMode {

    Follower follower;
    EeshMechanism m;

    public double wristpickup = -40;

    public class FollowerUpdate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            follower.update();
            m.worm.setPow2();
            m.slide.setPow2();
            m.update();
            return true;
        }
    }

    public Action follow(Path a) {
        return new InstantAction(() -> follower.followPath(a));
    }

    public Action followUntilDone(Path a) {
        return new FollowUntilDone(a);
    }

    public Action followUntilDoneChain(PathChain a) {
        return new FollowUntilDoneChain(a);
    }

    public class FollowUntilDone implements Action {

        Path a;
        boolean called = false;

        public FollowUntilDone(Path a)  {
            this.a = a;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!called) {
                follower.followPath(a);
                called = true;
            }
            return follower.isBusy();
        }
    }

    public class FollowUntilDoneChain implements Action {

        PathChain a;
        boolean called = false;

        public FollowUntilDoneChain(PathChain a)  {
            this.a = a;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!called) {
                follower.followPath(a);
                called = true;
            }
            return follower.isBusy();
        }
    }

    @Override
    public void runOpMode() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        m = new EeshMechanism(hardwareMap);
        final Pose dropPoseNew = new Pose(29.0915, 132.4722, 0);
        final Pose startPose = new Pose(8, 110.9, Math.toRadians(-90));
        final Pose dropFirstTwoPose = new Pose(12.5, 114.4);
        final Pose dropLastThree = new Pose(30.882, 118.6709);
        Constants.setConstants(FConstants.class, LConstants.class);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        Path dropPreload = new Path(
                new BezierLine(
                        new Point(startPose),
                        new Point(dropFirstTwoPose)
                )
        );
        dropPreload.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-67.3));

     /*   Path pickupTeammate =  new Path(
                new BezierLine(
                        new Point(dropFirstTwoPose),
                        new Point(9.4389, 105)
                )
        );
        pickupTeammate.setConstantHeadingInterpolation(Math.toRadians(-90));

        Path pickupTeammate2 = new Path(
                new BezierLine(
                        new Point(9.4389, 105),
                        new Point(9.4389, 92.1)
                )
        );
        pickupTeammate.setConstantHeadingInterpolation(Math.toRadians(-90));

        Path scoreTeammate = new Path(
                new BezierLine(
                        new Point(9.4389, 93.3),
                        new Point(new Pose(11.5427, 113.044))
                )
        );
        scoreTeammate.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-67.3));*/


        Path pickup10 = new Path(
                new BezierLine(
                        new Point(dropFirstTwoPose),
                        //new Point(11.5427, 117.4),
                        new Point(15, 117.5)//subtrat 0.5
                )
        );
        pickup10.setLinearHeadingInterpolation(Math.toRadians(-67.3), Math.toRadians(0));

        Path pickup1 = new Path(
                new BezierLine(
                        new Point(15, 117.5),//subtrat 0.5
                        //new Point(11.5427, 117.4),
                        new Point(23.5, 118.9)//subtrat 0.5
                )
        );
        pickup1.setConstantHeadingInterpolation(Math.toRadians(0));

        Path pickup12 = new Path(
                new BezierLine(
                        new Point(23.5, 118.9),//subtrat 0.5
                        new Point(30, 118.9)//subtrat 0.5

                )
        );
        pickup12.setConstantHeadingInterpolation(Math.toRadians(0));


        Path score1 = new Path(
                new BezierLine(
                        new Point(30, 118.9),//subtrat 0.5
                        new Point(new Pose(24.35, 119.2))

                )
        );
        score1.setLinearHeadingInterpolation(0, 5.5078);

        Path pickup20 = new Path(
                new BezierLine(
                        new Point(new Pose(24.25, 119.2)),
                        new Point(14, 126.1)//subtrat 1
                )
        );
        pickup20.setLinearHeadingInterpolation(5.5078, 0);

        Path pickup21 = new Path(
                new BezierLine(
                        new Point(14, 126.1),//subtrat 1
                        new Point(19, 128.3)//subtrat 1
                )
        );
        pickup21.setConstantHeadingInterpolation(0);

        Path pickup22 = new Path(
                new BezierLine(
                        new Point(19, 128.3),//subtrat 1
                        new Point(25.5, 128.3)//subtrat 1
                )
        );
        pickup22.setConstantHeadingInterpolation(Math.toRadians(0));

        Path score2 = new Path(
                new BezierLine(
                        new Point(25.5, 128.3),//subtrat 1
                        new Point(new Pose(23.75, 122.7))
                )
        );
        score2.setLinearHeadingInterpolation(0, 5.5078);


        Path pickup3 = new Path(
                new BezierLine(
                        new Point(new Pose(23.75, 122.7)),
                        new Point(21.2, 126)
                )
        );
        pickup3.setLinearHeadingInterpolation(5.5078, 0.469);

        Path pickup32 = new Path(
                new BezierLine(
                        new Point(21.2, 126),
                        new Point(26, 128.3)
                )
        );
        pickup32.setConstantHeadingInterpolation(0.469);

        Path pickup33 = new Path(
                new BezierLine(
                        new Point(26, 128.3),
                        new Point(30, 128.3)
                )
        );
        pickup33.setConstantHeadingInterpolation(0.469);


        Path score3 = new Path(
                new BezierLine(
                        new Point(30, 128.3),
                        new Point(25.1, 122)
                )
        );
        score3.setLinearHeadingInterpolation(0.469, 5.5078);







        Path pickupTeammate =  new Path(
                new BezierLine(
                        new Point(25.25, 120),
                        new Point(15.2, 118.7)
                )
        );
        pickupTeammate.setLinearHeadingInterpolation(5.5078, Math.toRadians(-90));

        Path pickupTeammate2 = new Path(
                new BezierLine(
                        new Point(15.2, 118.7),
                        new Point(13, 115.7)
                )
        );
        pickupTeammate2.setConstantHeadingInterpolation(Math.toRadians(-90));

        Path scoreTeammate = new Path(
                new BezierLine(
                        new Point(13, 115.7),
                        new Point(15.5, 114.5)
                )
        );
        scoreTeammate.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-60));









        Path pickupExtra =  new Path(
                new BezierCurve(
                        new Point(20.75, 118.25),
                        new Point(35.8453, 61.4782)
                )
        );
        pickupExtra.setLinearHeadingInterpolation(5.5078, 4.7443);

        Path pickupExtra2 = new Path(
                new BezierCurve(
                        new Point(35.8453, 61.4782),
                        new Point(25.5667, 36)
                )
        );
        pickupExtra2.setLinearHeadingInterpolation(4.7443, 5.418);
        Path pickupExtra3 = new Path(
                new BezierCurve(
                        new Point(25.5667, 36),
                        new Point(25.5667, 35)
                )
        );
        pickupExtra3.setConstantHeadingInterpolation(5.418);

        PathChain pickupExtraChain = new PathChain(
                pickupExtra,
                pickupExtra2,
                pickupExtra3
        );


        Path scoreExtra1 = new Path(
                new BezierLine(
                        new Point(25.5667, 34),
                        new Point(29, 61.4782)
                )
        );
        scoreExtra1.setLinearHeadingInterpolation(5.418, 4.7443);

        Path scoreExtra2 = new Path(
                new BezierLine(
                        new Point(29, 61.4782),
                        new Point(20.75, 118.25)
                )
        );
        scoreExtra2.setLinearHeadingInterpolation(4.7443, 5.5078);

        PathChain scoreExtraChain = new PathChain(
                scoreExtra1,
                scoreExtra2
        );


        waitForStart();
        if(isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                        new FollowerUpdate(),
                        new SequentialAction(
                                m.startIntake(),
                                m.worm.autoMove(1300),
                                followUntilDone(dropPreload),
                                //new SleepAction(0.5),
                                m.worm.waitUntilDone(),
                                //new SleepAction(0.4),
                                new InstantAction(() -> m.slide.runToPos(2420)),
                                new InstantAction(() -> m.setChosenAngle(150)),
                                m.slide.waitUntilDone(),
                                //new SleepAction(0.24),
                                m.drop(),
                                new SleepAction(0.25),
                                new InstantAction(() -> m.setChosenAngle(wristpickup)),
                                m.slide.liftBottom(),
                                m.slide.waitUntilDone(),
                                m.startIntake(),
                                m.worm.autoMove(-1210),
                                followUntilDone(pickup10),
                                //new SleepAction(0.6),
                                //new SleepAction(0.4),
                                new SleepAction(0.5),
                                m.worm.waitUntilDone(),
                                followUntilDone(pickup1),
                                followUntilDone(pickup12),
                                //new SleepAction(1),
                                m.worm.autoMove(1295),
                                new InstantAction(() -> m.setChosenAngle(150)),
                                followUntilDone(score1),
                                m.worm.waitUntilDone(),
                                m.slide.autoMove(2368),
                                new InstantAction(() -> m.setChosenAngle(150)),
                                m.slide.waitUntilDone(),
                                //new SleepAction(0.24),
                                m.drop(),
                                new SleepAction(0.25),
                                new InstantAction(() -> m.setChosenAngle(wristpickup)),
                                //new SleepAction(0.25),
                                m.slide.liftBottom(),
                                m.slide.waitUntilDone(),
                                m.startIntake(),
                                m.worm.autoMove(-1210),





                                followUntilDone(pickup20),
                                new SleepAction(0.5),
                                m.worm.waitUntilDone(),
                                //new SleepAction(0.25),
                                followUntilDone(pickup21),
                                followUntilDone(pickup22),
                                m.worm.autoMove(1295),
                                new InstantAction(() -> m.setChosenAngle(150)),
                                followUntilDone(score2),
                                m.worm.waitUntilDone(),
                                m.slide.autoMove(2368),
                                new InstantAction(() -> m.setChosenAngle(150)),
                                m.slide.waitUntilDone(),
                                //new SleepAction(0.24),
                                m.drop(),
                                new SleepAction(0.25),
                                new InstantAction(() -> m.setChosenAngle(wristpickup)),
                                //new SleepAction(0.25),
                                m.slide.liftBottom(),
                                m.slide.waitUntilDone(),
                                m.startIntake(),
                                m.worm.autoMove(-1210),


                                followUntilDone(pickup3),
                                new SleepAction(0.5),
                                m.worm.waitUntilDone(),
                                followUntilDone(pickup32),
                                followUntilDone(pickup33),
                                //new SleepAction(0.5),
                                m.worm.autoMove(1295),
                                new InstantAction(() -> m.setChosenAngle(150)),
                                followUntilDone(score3),
                                m.worm.waitUntilDone(),
                                m.slide.autoMove(2368),
                                new InstantAction(() -> m.setChosenAngle(150)),
                                m.slide.waitUntilDone(),
                                //new SleepAction(0.24),
                                m.drop(),
                                new SleepAction(0.25),
                                new InstantAction(() -> m.setChosenAngle(wristpickup)),
                                //new SleepAction(0.25),
                                m.slide.liftBottom(),
                                m.slide.waitUntilDone(),
                                m.startIntake(),
                                m.worm.autoMove(-1210),




                                m.worm.waitUntilDone(),
                                followUntilDone(pickupTeammate),
                                followUntilDone(pickupTeammate2),
                                //new SleepAction(0.5),
                                m.worm.autoMove(1295),
                                new InstantAction(() -> m.setChosenAngle(150)),
                                followUntilDone(scoreTeammate),
                                m.worm.waitUntilDone(),
                                m.slide.autoMove(2368),
                                new InstantAction(() -> m.setChosenAngle(150)),
                                m.slide.waitUntilDone(),
                                new SleepAction(0.24),
                                m.drop()
                        )
                ));

    }
}
