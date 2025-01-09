package org.firstinspires.ftc.teamcode.autos;



import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import org.firstinspires.ftc.teamcode.wrappers.EeshMechanism;


import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous()
public class BZC extends LinearOpMode {

    Follower follower;
    EeshMechanism m;

    public double wristpickup = 0.602;


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

        final Pose startPose = new Pose(8, 110.9, Math.toRadians(-90));
        final Pose scorePose = new Pose(27, 130, 6.28);

        double xScoreInc = 1.75;
        double yScoreInc = -1.25;

        final Pose control1Pose = new Pose(10, 125, 6.28);
        final Pose pickup1Pose = new Pose(20.9, 119.5, 6.28);
        final Pose score1Pose = new Pose(27 + 1.75, 130 + yScoreInc, 6.28);

        final Pose control2Pose = new Pose(14, 132, 6.28);
        final Pose pickup2Pose = new Pose(26, 127, 6.28);
        final Pose score2Pose = new Pose(27, 130 + yScoreInc, 6.28);

        final Pose control3Pose = new Pose(10, 137, 6.28);
        final Pose pickup3Pose1 = new Pose(29, 123.25, Math.toRadians(43.4));
        final Pose pickup3Pose2 = new Pose(30, 123.25, Math.toRadians(43.4));
        final Pose score3Pose = new Pose(27 + 4.5, 130 + yScoreInc, 6.28);



        Constants.setConstants(FConstants.class, LConstants.class);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        Path dropPreload = new Path(
                new BezierLine(
                        new Point(startPose),
                        new Point(scorePose)
                )
        );
        dropPreload.setLinearHeadingInterpolation(Math.toRadians(-90), 6.28);

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


        Path pickup1 = new Path(
                new BezierCurve(
                        new Point(scorePose),
                        new Point(control1Pose),
                        new Point(pickup1Pose)//subtrat 0.5
                )
        );
        pickup1.setConstantHeadingInterpolation(6.28);


        Path score1 = new Path(
                new BezierLine(
                        new Point(pickup1Pose),
                        new Point(score1Pose)

                )
        );
        score1.setConstantHeadingInterpolation(6.28);

        Path pickup2 = new Path(
                new BezierCurve(
                        new Point(scorePose),
                        new Point(control2Pose),
                        new Point(pickup2Pose)//subtrat 1
                )
        );
        pickup2.setConstantHeadingInterpolation(6.28);

        Path score2 = new Path(
                new BezierLine(
                        new Point(pickup2Pose),//subtrat 1
                        new Point(score2Pose)
                )
        );
        score2.setConstantHeadingInterpolation(6.28);




        Path pickup31 = new Path(
                new BezierCurve(
                        new Point(scorePose),
                        new Point(control3Pose),
                        new Point(pickup3Pose1)
                )
        );
        pickup31.setLinearHeadingInterpolation(6.28, Math.toRadians(43.4));

        Path pickup32 = new Path(
                new BezierLine(
                        new Point(pickup3Pose1),
                        new Point(pickup3Pose2)
                )
        );
        pickup32.setConstantHeadingInterpolation(Math.toRadians(43.4));

        Path score3 = new Path(
                new BezierLine(
                        new Point(pickup3Pose2),
                        new Point(score3Pose)
                )
        );
        score3.setLinearHeadingInterpolation(Math.toRadians(43.4), 6.28);







        Path pickupTeammate =  new Path(
                new BezierLine(
                        new Point(scorePose),
                        new Point(8.5, 98)
                )
        );
        pickupTeammate.setLinearHeadingInterpolation(0.9363, Math.toRadians(-90));

        Path pickupTeammate2 = new Path(
                new BezierLine(
                        new Point(8.5, 98),
                        new Point(8.5, 92.35)
                )
        );
        pickupTeammate.setConstantHeadingInterpolation(Math.toRadians(-90));

        Path scoreTeammate = new Path(
                new BezierLine(
                        new Point(8.5, 92.35),
                        new Point(10, 110.5)
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
                                m.worm.autoMove(1380),
                                followUntilDone(dropPreload),
                                m.worm.waitUntilDone(),
                                new SleepAction(0.4),
                                m.slide.autoMove(2365),
                                new ParallelAction(
                                        //new SleepAction(0.4),
                                        new InstantAction(() -> m.setChosenAngle(150))
                                ),
                                m.slide.waitUntilDone(),
                                new SleepAction(0.24),
                                m.drop(),
                                new SleepAction(0.25),
                                new InstantAction(() -> m.setWrist(wristpickup)),
                                m.slide.liftBottom(),
                                m.slide.waitUntilDone(),
                                m.startIntake(),

                                m.worm.autoMove(-1030),
                                //new SleepAction(0.6),
                                //new SleepAction(0.4),
                                // m.worm.waitUntilDone(),
                                followUntilDone(pickup1),
                                //followUntilDone(pickup12),
                                //new SleepAction(1),
                                m.worm.autoMove(1295),
                                new InstantAction(() -> m.setChosenAngle(150)),
                                followUntilDone(score1),
                                m.worm.waitUntilDone(),
                                m.slide.autoMove(2368),
                                new InstantAction(() -> m.setChosenAngle(150)),
                                m.slide.waitUntilDone(),
                                new SleepAction(0.24),
                                m.drop(),
                                new SleepAction(0.25),
                                new InstantAction(() -> m.setWrist(wristpickup)),
                                //new SleepAction(0.25),
                                m.slide.liftBottom(),
                                m.slide.waitUntilDone(),
                                m.startIntake(),
                                m.worm.autoMove(-1030),





                                // m.worm.waitUntilDone(),
                                //new SleepAction(0.25),
                                followUntilDone(pickup2),
                                m.worm.autoMove(1295),
                                new InstantAction(() -> m.setChosenAngle(150)),
                                followUntilDone(score2),
                                m.worm.waitUntilDone(),
                                m.slide.autoMove(2368),
                                new InstantAction(() -> m.setChosenAngle(150)),
                                m.slide.waitUntilDone(),
                                new SleepAction(0.24),
                                m.drop(),
                                new SleepAction(0.25),
                                new InstantAction(() -> m.setWrist(wristpickup)),
                                //new SleepAction(0.25),
                                m.slide.liftBottom(),
                                m.slide.waitUntilDone(),
                                m.startIntake(),
                                m.worm.autoMove(-1030),


                                // m.worm.waitUntilDone(),
                                followUntilDone(pickup31),
                                followUntilDone(pickup32),
                                //new SleepAction(0.5),
                                m.worm.autoMove(1295),
                                new InstantAction(() -> m.setChosenAngle(150)),
                                followUntilDone(score3),
                                m.worm.waitUntilDone(),
                                m.slide.autoMove(2368),
                                new InstantAction(() -> m.setChosenAngle(150)),
                                m.slide.waitUntilDone(),
                                new SleepAction(0.24),
                                m.drop(),
                                new SleepAction(0.25),
                                new InstantAction(() -> m.setWrist(wristpickup)),
                                //new SleepAction(0.25),
                                m.slide.liftBottom(),
                                m.slide.waitUntilDone(),
                                m.startIntake(),
                                m.worm.autoMove(-1030)/*,




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
                                m.drop(),
                                new SleepAction(0.25),
                                new InstantAction(() -> m.setWrist(wristpickup)),
                                //new SleepAction(0.25),
                                m.slide.liftBottom(),
                                m.slide.waitUntilDone(),
                                m.startIntake(),
                                m.worm.autoMove(-1030),


                                m.worm.waitUntilDone(),
                                //new InstantAction(() -> m.setWrist(0.83)),
                                followUntilDoneChain(pickupExtraChain),
                                //new SleepAction(0.5),
                                m.worm.autoMove(1295),
                                new InstantAction(() -> m.setChosenAngle(150)),
                                followUntilDoneChain(scoreExtraChain),
                                m.worm.waitUntilDone(),
                                m.slide.autoMove(2368),
                                new InstantAction(() -> m.setChosenAngle(150)),
                                m.slide.waitUntilDone(),
                                new SleepAction(0.24),
                                m.drop(),
                                new SleepAction(0.25),
                                new InstantAction(() -> m.setWrist(wristpickup)),
                                //new SleepAction(0.25),
                                m.slide.liftBottom(),
                                m.slide.waitUntilDone(),
                                m.startIntake(),
                                m.worm.autoMove(-1030)*/


                        )
                ));

    }
}