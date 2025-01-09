package org.firstinspires.ftc.teamcode.drive.writtenCode.auto;

import static org.firstinspires.ftc.teamcode.drive.writtenCode.auto.AutoSpecimen.STROBOT.DELIVER_PICKUP3;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.auto.AutoSpecimen.STROBOT.END_AUTO;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.auto.AutoSpecimen.STROBOT.GRAB_SPECIMEN1;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.auto.AutoSpecimen.STROBOT.GRAB_SPECIMEN2;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.auto.AutoSpecimen.STROBOT.GRAB_SPECIMEN3;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.auto.AutoSpecimen.STROBOT.GRAB_SPECIMEN4;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.auto.AutoSpecimen.STROBOT.PICKUP2;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.auto.AutoSpecimen.STROBOT.PICKUP3;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.auto.AutoSpecimen.STROBOT.SCORE_SPECIMEN1;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.auto.AutoSpecimen.STROBOT.SCORE_SPECIMEN2;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.auto.AutoSpecimen.STROBOT.SCORE_SPECIMEN3;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.auto.AutoSpecimen.STROBOT.SCORE_SPECIMEN4;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.auto.AutoSpecimen.STROBOT.SPECIMEN1;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.auto.AutoSpecimen.STROBOT.SPECIMEN2;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.auto.AutoSpecimen.STROBOT.SPECIMEN3;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.auto.AutoSpecimen.STROBOT.SPECIMEN4;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ClawController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ClawPositionController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ClawRotateController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ClimbController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.FourbarController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.LinkageController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.LinkageSlidesController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ScoreSystemController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.SlidesController;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Config
@Autonomous(group = "Auto")

public class AutoSpecimenPush extends LinearOpMode {



    public static double PRELOAD_LEFT_X = -38;
    public static double PRELOAD_LEFT_Y = 62;
    private void wait(int ms) {
        try{
            Thread.sleep(ms);
        }
        catch (InterruptedException ignored) {
            //nu face nimic
        }
    }


    enum STROBOT
    {
        START,
        PLACE_PRELOAD,
        PICKUP1, GRAB_PICKUP1, DELIVER_PICKUP1, PICKUP2, DELIVER_PICKUP2, GRAB_PICKUP2, PICKUP3, GRAB_PICKUP3, DELIVER_PICKUP3, END_AUTO
        ,SPECIMEN1,GRAB_SPECIMEN1, GRAB_SPECIMEN2, SCORE_SPECIMEN2, SPECIMEN2, SPECIMEN3, GRAB_SPECIMEN3, SCORE_SPECIMEN3, SPECIMEN4, GRAB_SPECIMEN4, SCORE_SPECIMEN4, SCORE_SPECIMEN1
    }
    ElapsedTime AutoTimer = new ElapsedTime();
    ElapsedTime TimerLeave = new ElapsedTime();
    ElapsedTime TimerPlacePreload = new ElapsedTime();
    ElapsedTime TimerPickup = new ElapsedTime();
    ElapsedTime TimerDeliver = new ElapsedTime();
    ElapsedTime TimerScore = new ElapsedTime();
    private double time_place_preload = 1.3;
    private double time_leave_preload = 0.5;
    private Follower follower;

    int linkage_target_position = 0;
    int claw_rotate_target = 0;
    int slides_target_position = 0;

    private Path  specimen1,specimen4,specimen4Score,specimen3,specimen3Score,specimen2,specimen2Score,specimen1Score,scorePreload,grabPickup1,deliver1,grabPickup2,deliver2,grabPickup3,deliver3,park;
    private PathChain pickup1Chain,pickup2Chain,pickup3Chain;
    public static double tunex=0;
    public static double tuney=0;
    final Pose startPose = new Pose(10, 63, Math.toRadians(0));
    final Pose preloadPose = new Pose(39.7, 73, Math.toRadians(0));

    final Pose pickup1Pose = new Pose(50, 33, Math.toRadians(0));
    final Point pickup1Point = new Point(23.392, 31.762, Point.CARTESIAN);
    final Point pickup1Point2 = new Point(36.697, 32.835, Point.CARTESIAN);
    final Pose deliver1Pose = new Pose(22,22,Math.toRadians(0));
    final Point deliver1Point = new Point(74.012, 18.246, Point.CARTESIAN);

    final Pose pickup2Pose = new Pose(62, 15, Math.toRadians(0));
    final Point pickup2Point = new Point(66.7, 35, Point.CARTESIAN);
    final Pose deliver2Pose = new Pose(21,14,Math.toRadians(0));

    final Pose pickup3Pose = new Pose(60, 8, Math.toRadians(0));
    final Point pickup3Point = new Point(65, 30, Point.CARTESIAN);
    final Pose deliver3Pose = new Pose(23,8,Math.toRadians(0));

    final Pose specimen1Pose = new Pose(11.4,35.5,Math.toRadians(0));
    final Point specimen1Point1 = new Point(31, 21, Point.CARTESIAN);
    final Point specimen1Point2 = new Point(28, 35.5, Point.CARTESIAN);

    final Pose specimen1ScorePose = new Pose(39.5, 68, Math.toRadians(0));

    final Pose specimen2Pose = new Pose(11.4,32.4,Math.toRadians(0));
    final Point specimen2Point1 = new Point(28, 52, Point.CARTESIAN);
    final Point specimen2Point2 = new Point(51, 29, Point.CARTESIAN);
    final Pose specimen2ScorePose = new Pose(40, 66, Math.toRadians(0));

    final Pose specimen3Pose = new Pose(11.4,32.4,Math.toRadians(0));
    final Point specimen3Point1 = new Point(28, 52, Point.CARTESIAN);
    final Point specimen3Point2 = new Point(51, 29, Point.CARTESIAN);
    final Pose specimen3ScorePose = new Pose(40, 64, Math.toRadians(0));

    final Pose specimen4Pose = new Pose(11.4,32.4,Math.toRadians(0));
    final Point specimen4Point1 = new Point(28, 52, Point.CARTESIAN);
    final Point specimen4Point2 = new Point(51, 29, Point.CARTESIAN);
    final Pose specimen4ScorePose = new Pose(40, 63, Math.toRadians(0));

    final Pose parkPose =new Pose(13,32.4, Math.toRadians(0));


    @Override
    public void runOpMode() throws InterruptedException {
        RobotMap robot = new RobotMap(hardwareMap);
        DigitalChannel beam = robot.beam;
        ClawController claw = new ClawController(robot);
        ClawPositionController clawPosition = new ClawPositionController(robot);
        ClawRotateController clawRotate = new ClawRotateController(robot);
        FourbarController fourbar = new FourbarController(robot);
        LinkageController linkage = new LinkageController(robot);
        SlidesController slides = new SlidesController(robot);
        ScoreSystemController scoreSystem = new ScoreSystemController(claw,clawRotate,fourbar,clawPosition);
        LinkageSlidesController linkageSlides = new LinkageSlidesController(linkage,slides);
        ClimbController climb = new ClimbController(robot);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.INIT_AUTO;

        scoreSystem.update();
        linkageSlides.update();
        claw.update();
        clawPosition.update();
        clawRotate.update(ClawRotateController.init_position,0);
        fourbar.update();
        climb.update();
        follower.update();
        linkage.update(LinkageController.init_position,linkage_target_position);
        slides.update(SlidesController.init_position,slides_target_position);

        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(preloadPose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(),preloadPose.getHeading());

        grabPickup1 = new Path(new BezierCurve(new Point(preloadPose), pickup1Point, pickup1Point2, new Point(pickup1Pose)));
        grabPickup1.setLinearHeadingInterpolation(preloadPose.getHeading(),pickup1Pose.getHeading());

        deliver1 = new Path(new BezierCurve(new Point(pickup1Pose),deliver1Point,new Point(deliver1Pose)));
        deliver1.setLinearHeadingInterpolation(pickup1Pose.getHeading(), deliver1Pose.getHeading());

        grabPickup2 = new Path(new BezierCurve(new Point(deliver1Pose), pickup2Point, new Point(pickup2Pose)));
        grabPickup2.setLinearHeadingInterpolation(deliver1Pose.getHeading(),pickup2Pose.getHeading());

        deliver2 = new Path(new BezierCurve(new Point(pickup2Pose),deliver1Point,new Point(deliver2Pose)));
        deliver2.setLinearHeadingInterpolation(pickup2Pose.getHeading(), deliver2Pose.getHeading());

        grabPickup3 = new Path(new BezierLine(new Point(deliver2Pose), new Point(pickup3Pose)));
        grabPickup3.setLinearHeadingInterpolation(deliver2Pose.getHeading(), pickup3Pose.getHeading());

        deliver3 = new Path(new BezierLine(new Point(pickup3Pose), new Point(deliver3Pose)));
        deliver3.setLinearHeadingInterpolation(pickup3Pose.getHeading(), deliver3Pose.getHeading());

        pickup1Chain = follower.pathBuilder()
                .addPath(grabPickup1)
                .addPath(deliver1)
                .build();

        pickup2Chain = follower.pathBuilder()
                .addPath(grabPickup2)
                .addPath(deliver2)
                .build();

        pickup3Chain = follower.pathBuilder()
                .addPath(grabPickup3)
                .addPath(deliver3)
                .build();

        specimen1 = new Path(new BezierCurve(
                new Point(deliver3Pose),
                specimen1Point1,
                specimen1Point2,
                new Point(specimen1Pose)
        ));
        specimen1.setLinearHeadingInterpolation(deliver3Pose.getHeading(), specimen1Pose.getHeading());

        specimen1Score=new Path(new BezierLine(new Point(specimen1Pose), new Point(specimen1ScorePose)));
        specimen1Score.setLinearHeadingInterpolation(specimen1Pose.getHeading(), specimen1ScorePose.getHeading());

        specimen2 = new Path(new BezierCurve(
                new Point(specimen1ScorePose),
                specimen2Point1,
                specimen2Point2,
                new Point(specimen2Pose)
        ));
        specimen2.setLinearHeadingInterpolation(specimen1ScorePose.getHeading(), specimen2Pose.getHeading());

        specimen2Score=new Path(new BezierLine(new Point(specimen2Pose), new Point(specimen2ScorePose)));
        specimen2Score.setLinearHeadingInterpolation(specimen2Pose.getHeading(), specimen2ScorePose.getHeading());

        specimen3 = new Path(new BezierCurve(
                new Point(specimen2ScorePose),
                specimen3Point1,
                specimen3Point2,
                new Point(specimen3Pose)
        ));
        specimen3.setLinearHeadingInterpolation(specimen2ScorePose.getHeading(), specimen3Pose.getHeading());

        specimen3Score=new Path(new BezierLine(new Point(specimen3Pose), new Point(specimen3ScorePose)));
        specimen3Score.setLinearHeadingInterpolation(specimen3Pose.getHeading(), specimen3ScorePose.getHeading());

        specimen4 = new Path(new BezierCurve(
                new Point(specimen3ScorePose),
                specimen4Point1,
                specimen4Point2,
                new Point(specimen4Pose)
        ));
        specimen4.setLinearHeadingInterpolation(specimen3ScorePose.getHeading(), specimen4Pose.getHeading());

        specimen4Score=new Path(new BezierLine(new Point(specimen4Pose), new Point(specimen4ScorePose)));
        specimen4Score.setLinearHeadingInterpolation(specimen4Pose.getHeading(), specimen4ScorePose.getHeading());

        park = new Path(new BezierLine(new Point(specimen4ScorePose),new Point(parkPose)));
        park.setLinearHeadingInterpolation(specimen4ScorePose.getHeading(), parkPose.getHeading());
        boolean ok=false;

        STROBOT status = STROBOT.START;
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested())
        {
            int slides_current_position = robot.linkage.getCurrentPosition();
            int linkage_current_position = linkage.encoderLinkage.getCurrentPosition();
            switch (status) {
                case START: {
                    AutoTimer.reset();
                    TimerPlacePreload.reset();
                    follower.followPath(scorePreload,false);
                    scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.RUNG;
                    linkageSlides.currentStatus = LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG;
                    status = STROBOT.PLACE_PRELOAD;
                    break;
                }
                case PLACE_PRELOAD: {
                    if(follower.getPose().getX() > (preloadPose.getX() - 1) && follower.getPose().getY() > (preloadPose.getY() - 1)
                            && TimerPlacePreload.seconds()>time_place_preload) {
                        linkageSlides.currentStatus = LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE;
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.OPEN_RUNG;
                        TimerLeave.reset();
                        status = STROBOT.PICKUP1;
                    }
                    break;
                }
                case PICKUP1:
                {
                    if(TimerLeave.seconds()>time_leave_preload)
                    {
//                        follower.followPath(grabPickup1, false);
                        follower.followPath(pickup1Chain,true);
                        status=STROBOT.GRAB_PICKUP1;
                    }
                    break;
                }
                case GRAB_PICKUP1:
                {
                    if(TimerLeave.seconds()>time_leave_preload+0.5)
                    {
                        slides.currentStatus= SlidesController.SlidesStatus.INIT;
                        linkage.currentStatus = LinkageController.LinkageStatus.INIT;
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.INIT;
                        status=STROBOT.DELIVER_PICKUP1;
                    }
                    break;
                }
                case DELIVER_PICKUP1:
                {
                    TimerLeave.reset();
                    status = STROBOT.PICKUP2;
                    break;
                }
                case PICKUP2:
                {
                    if(TimerLeave.seconds()>3)
                    {
//                        follower.followPath(grabPickup1, false);
                        follower.followPath(pickup2Chain,true);
                        status=STROBOT.GRAB_PICKUP2;
                    }
                    break;
                }
                case GRAB_PICKUP2:
                {
                    TimerDeliver.reset();
                    status=STROBOT.DELIVER_PICKUP2;
                    break;
                }
                case DELIVER_PICKUP2:
                {
                    if(TimerDeliver.seconds()>0.75)
                    {
                        claw.currentStatus = ClawController.ClawStatus.OPEN;
                        TimerLeave.reset();
                        status=STROBOT.PICKUP3;
                    }
                    break;
                }
                case PICKUP3:
                {
                    if(TimerLeave.seconds()>2)
                    {
//                        follower.followPath(grabPickup1, false);
                        follower.followPath(pickup3Chain,true);
                        status=STROBOT.GRAB_PICKUP3;
                    }
                    break;
                }
                case GRAB_PICKUP3:
                {
                    TimerDeliver.reset();
                    status=STROBOT.DELIVER_PICKUP3;
                    break;
                }
                case DELIVER_PICKUP3:
                {
                    if(TimerDeliver.seconds()>0.75)
                    {
                        claw.currentStatus = ClawController.ClawStatus.OPEN;
                        TimerLeave.reset();
                        status=STROBOT.SPECIMEN1;
                    }
                    break;
                }
                case SPECIMEN1:
                {
                    if(TimerLeave.seconds()>2)
                    {
                        follower.followPath(specimen1);
                        status = STROBOT.GRAB_SPECIMEN1;
                        TimerLeave.reset();
                    }
                    break;
                }
                case GRAB_SPECIMEN1:
                {
                    if(!follower.isBusy() && beam.getState()==false)
                    {
                        if(ok==false) {
                            scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.LOWER_FOURBAR;
                            ok=true;
                        }
                        if(claw.currentStatus == ClawController.ClawStatus.CLOSE && TimerLeave.seconds()>0.8)
                        {
                            status = STROBOT.SCORE_SPECIMEN1;
                            ok=false;
                            follower.followPath(specimen1Score);
                            TimerScore.reset();
                        }
                    }
                    break;
                }
                case SCORE_SPECIMEN1:
                {
                    if(TimerScore.seconds()>0.4 && ok==false)
                    {
                        scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.RUNG;
                        linkageSlides.currentStatus = LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG;
                        ok=true;
                    }
                    if(!follower.isBusy() && linkageSlides.currentStatus== LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG)
                    {
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE;
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.OPEN_RUNG;
                        TimerLeave.reset();
                    }
                    if(!follower.isBusy()  && linkageSlides.currentStatus== LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE && TimerLeave.seconds()>0.25)
                    {
                        ok=false;
                        status=STROBOT.SPECIMEN2;
                        TimerLeave.reset();
                    }
                    break;
                }
                case SPECIMEN2:
                {
                    if(TimerLeave.seconds()>0)
                    {
                        follower.followPath(specimen2);
                        ok=false;
                        status = STROBOT.GRAB_SPECIMEN2;
                        TimerLeave.reset();
                    }
                    break;
                }
                case GRAB_SPECIMEN2:
                {
                    if(TimerLeave.seconds()>0.3 && linkageSlides.currentStatus!= LinkageSlidesController.LinkageSlidesStatus.INIT)
                    {
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.INIT;
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.INIT;
                    }
                    if(!follower.isBusy() && beam.getState()==false)
                    {
                        if(ok==false) {
                            scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.LOWER_FOURBAR;
                            ok=true;
                        }
                        if(claw.currentStatus == ClawController.ClawStatus.CLOSE && TimerLeave.seconds()>1.5)
                        {
                            status = STROBOT.SCORE_SPECIMEN2;
                            ok=false;
                            follower.followPath(specimen2Score);
                            TimerScore.reset();
                        }
                    }
                    break;
                }
                case SCORE_SPECIMEN2:
                {
                    if(TimerScore.seconds()>0.4 && ok==false)
                    {
                        scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.RUNG;
                        linkageSlides.currentStatus = LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG;
                        ok=true;
                    }
                    if(!follower.isBusy() && linkageSlides.currentStatus== LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG)
                    {
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE;
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.OPEN_RUNG;
                        TimerLeave.reset();
                    }
                    if(!follower.isBusy()  && linkageSlides.currentStatus== LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE && TimerLeave.seconds()>0.25)
                    {
                        ok=false;
                        status=STROBOT.SPECIMEN3;
                        TimerLeave.reset();
                    }
                    break;
                }
                case SPECIMEN3:
                {
                    if(TimerLeave.seconds()>0)
                    {
                        follower.followPath(specimen3);
                        ok=false;
                        status = STROBOT.GRAB_SPECIMEN3;
                        TimerLeave.reset();
                    }
                    break;
                }
                case GRAB_SPECIMEN3:
                {
                    if(TimerLeave.seconds()>0.3 && linkageSlides.currentStatus!= LinkageSlidesController.LinkageSlidesStatus.INIT)
                    {
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.INIT;
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.INIT;
                    }
                    if(!follower.isBusy() && beam.getState()==false)
                    {
                        if(ok==false) {
                            scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.LOWER_FOURBAR;
                            ok=true;
                        }
                        if(claw.currentStatus == ClawController.ClawStatus.CLOSE && TimerLeave.seconds()>1.5)
                        {
                            status = STROBOT.SCORE_SPECIMEN3;
                            ok=false;
                            follower.followPath(specimen3Score);
                            TimerScore.reset();
                        }
                    }
                    break;
                }
                case SCORE_SPECIMEN3:
                {
                    if(TimerScore.seconds()>0.4 && ok==false)
                    {
                        scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.RUNG;
                        linkageSlides.currentStatus = LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG;
                        ok=true;
                    }
                    if(!follower.isBusy() && linkageSlides.currentStatus== LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG)
                    {
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE;
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.OPEN_RUNG;
                        TimerLeave.reset();
                    }
                    if(!follower.isBusy()  && linkageSlides.currentStatus== LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE && TimerLeave.seconds()>0.25)
                    {
                        ok=false;
                        TimerLeave.reset();
                        status=STROBOT.SPECIMEN4;
                    }
                    break;
                }
                case SPECIMEN4:
                {
                    if(TimerLeave.seconds()>0)
                    {
                        follower.followPath(specimen4);
                        ok=false;
                        status = STROBOT.GRAB_SPECIMEN4;
                        TimerLeave.reset();
                    }
                    break;
                }
                case GRAB_SPECIMEN4:
                {
                    if(TimerLeave.seconds()>0.3 && linkageSlides.currentStatus!= LinkageSlidesController.LinkageSlidesStatus.INIT)
                    {
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.INIT;
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.INIT;
                    }
                    if(!follower.isBusy() && beam.getState()==false)
                    {
                        if(ok==false) {
                            scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.LOWER_FOURBAR;
                            ok=true;
                        }
                        if(claw.currentStatus == ClawController.ClawStatus.CLOSE && TimerLeave.seconds()>1.5)
                        {
                            status = STROBOT.SCORE_SPECIMEN4;
                            ok=false;
                            follower.followPath(specimen4Score);
                            TimerScore.reset();
                        }
                    }
                    break;
                }
                case SCORE_SPECIMEN4:
                {
                    if(TimerScore.seconds()>0.4 && ok==false)
                    {
                        scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.RUNG;
                        linkageSlides.currentStatus = LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG;
                        ok=true;
                    }
                    if(!follower.isBusy() && linkageSlides.currentStatus== LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG)
                    {
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE;
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.OPEN_RUNG;
                        TimerLeave.reset();
                    }
                    if(!follower.isBusy()  && linkageSlides.currentStatus== LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE && TimerLeave.seconds()>0.25)
                    {
                        ok=false;
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.INIT;
                        follower.followPath(park);
                        status=STROBOT.END_AUTO;
                    }
                    break;
                }
            }
            linkage.update(linkage_current_position,linkage_target_position);
            slides.update(slides_current_position,slides_target_position);
            claw.update();
            fourbar.update();
            clawPosition.update();
            clawRotate.update(claw_rotate_target, 0);
            scoreSystem.update();
            climb.update();
            follower.update();
            linkageSlides.update();
            telemetry.addData("status", status);
            telemetry.addData("slidescurrent", slides_current_position);
            telemetry.addData("clawposition",clawPosition.currentStatus);
            telemetry.addData("scoresysten",scoreSystem.currentStatus);
            telemetry.addData("Current Traj", follower.getCurrentPath());
            telemetry.addData("beam",beam.getState());
            telemetry.addData("ok", ok);
            telemetry.addData("pose", follower.getPose());
            telemetry.update();
        }
    }

}