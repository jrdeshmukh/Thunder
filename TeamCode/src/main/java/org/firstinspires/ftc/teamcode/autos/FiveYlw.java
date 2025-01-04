package org.firstinspires.ftc.teamcode.autos;



import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
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
public class FiveYlw extends LinearOpMode {

    Follower follower;
    EeshMechanism m;

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



    @Override
    public void runOpMode() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        m = new EeshMechanism(hardwareMap);
        final Pose startPose = new Pose(8, 110.9, Math.toRadians(-90));
        final Pose dropFirstTwoPose = new Pose(11.5427, 114.044);
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

        Path pickupTeammate =  new Path(
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
        scoreTeammate.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-67.3));


        Path pickup1 = new Path(
                new BezierLine(
                        new Point(new Pose(11.5427, 113.044)),
                        new Point(11.5427, 117.4)
                )
        );
        pickup1.setLinearHeadingInterpolation(Math.toRadians(-67.3), Math.toRadians(0));

        Path pickup12 = new Path(
                new BezierLine(
                        new Point(11.5, 117.4),
        new Point(28, 118.5)

                )
        );
        pickup12.setConstantHeadingInterpolation(Math.toRadians(0));


        Path score1 = new Path(
                new BezierLine(
                        new Point(31, 120),
                        new Point(new Pose(25, 120))

                )
        );
        score1.setLinearHeadingInterpolation(0, 5.5078);

        Path pickup21 = new Path(
                new BezierLine(
                        new Point(new Pose(25, 120)),
                        new Point(25, 127.7)
                )
        );
        pickup21.setLinearHeadingInterpolation(5.5078, 0);

        Path pickup22 = new Path(
                new BezierLine(
                        new Point(25, 127.7),
                        new Point(31.14, 127.7)
                )
        );

        Path score2 = new Path(
                new BezierLine(
                        new Point(31.14, 127.7),
                        new Point(new Pose(25, 120))
                )
        );
        score2.setLinearHeadingInterpolation(0, 5.5078);







        waitForStart();
        if(isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                        new FollowerUpdate(),
                        new SequentialAction(
                                m.startIntake(),
                                follow(dropPreload),
                                m.worm.autoMove(1300),
                                new SleepAction(0.4),
                                m.slide.autoMove(2320),
                                new ParallelAction(
                                        //new SleepAction(0.4),
                                        new InstantAction(() -> m.setChosenAngle(200))
                                ),
                                m.slide.waitUntilDone(),
                                new SleepAction(0.4),
                                m.drop(),
                                new SleepAction(0.5),
                                m.slide.liftBottom(),
                                m.startIntake(),
                                m.setAngleAction(EeshMechanism.WRIST_PICKUP_ANGLE),
                                m.worm.autoMove(-1035),
                                m.worm.waitUntilDone(),
                                m.slide.waitUntilDone(),
                                new SleepAction(0.8),
                                followUntilDone(pickupTeammate),
                                followUntilDone(pickupTeammate2),
                                m.worm.autoMove(1300),
                                followUntilDone(scoreTeammate),
                                m.slide.autoMove(2330),
                                new ParallelAction(
                                        new SleepAction(1),
                                        new InstantAction(() -> m.setChosenAngle(200))
                                ),
                                m.slide.waitUntilDone(),
                                m.drop(),
                                new SleepAction(0.5),
                                m.setAngleAction(EeshMechanism.WRIST_PICKUP_ANGLE),
                                m.startIntake(),
                                new SleepAction(0.5),
                                m.slide.liftBottom(),
                                m.worm.autoMove(-1048),
                                new SleepAction(0.8),
                                followUntilDone(pickup1),
                                new SleepAction(0.5),
                                follow(pickup12),
                                new SleepAction(1),
                                m.worm.autoMove(1295),
                                new InstantAction(() -> m.setChosenAngle(200)),
                                followUntilDone(score1),
                                m.slide.autoMove(2368),
                                new InstantAction(() -> m.setChosenAngle(200)),
                                m.slide.waitUntilDone(),
                                m.drop(),
                                new SleepAction(0.5),
                                m.setAngleAction(EeshMechanism.WRIST_PICKUP_ANGLE),
                                new SleepAction(0.3),
                                m.slide.liftBottom(),
                                m.startIntake(),
                                m.worm.autoMove(-1040),
                                followUntilDone(pickup21),
                                followUntilDone(pickup22),
                                followUntilDone(score2),
                                m.worm.autoMove(1295),
                                new InstantAction(() -> m.setChosenAngle(200)),
                                m.slide.autoMove(2368),
                                new InstantAction(() -> m.setChosenAngle(200)),
                                m.slide.waitUntilDone(),
                                m.drop(),
                                new SleepAction(0.5),
                                m.setAngleAction(EeshMechanism.WRIST_PICKUP_ANGLE),
                                new SleepAction(0.3),
                                m.slide.liftBottom(),
                                m.startIntake(),
                                m.worm.autoMove(-1040)






                )
        ));

    }
}
