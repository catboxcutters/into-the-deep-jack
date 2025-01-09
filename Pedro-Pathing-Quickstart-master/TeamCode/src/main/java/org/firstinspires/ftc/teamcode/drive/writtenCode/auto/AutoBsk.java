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

public class AutoBsk extends LinearOpMode {


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
        PICKUP1, GRAB_PICKUP1, PICKUP2, GRAB_PICKUP2, PICKUP3, GRAB_PICKUP3, END_AUTO,
        SCORE_PICKUP1, SCORE_PICKUP2, SCORE_PICKUP3, SCORE_INTER1, SCORE_INTER2;
    }
    ElapsedTime AutoTimer = new ElapsedTime();
    ElapsedTime TimerLeave = new ElapsedTime();
    ElapsedTime TimerPlacePreload = new ElapsedTime();
    ElapsedTime TimerPickup = new ElapsedTime();
    ElapsedTime TimerScore = new ElapsedTime();
    private double time_place_preload = 1.3;
    private double time_leave_preload = 0.5;
    private Follower follower;

    int linkage_target_position = 0;
    int claw_rotate_target = 0;
    int slides_target_position = 0;

    private Path scorePreload ,pickup1,scorePickup1Inter,scorePickup1,pickup2,scorePickup2Inter,scorePickup2,park;
    private PathChain scorePickup1Chain;
    public static double tunex=0;
    public static double tuney=0;
    final Pose startPose = new Pose(10, 111, Math.toRadians(0));
    final Pose preloadPose = new Pose(11.5, 127, Math.toRadians(312));
    final Pose interPose = new Pose(27.2,121.7,Math.toRadians(315));
    final Pose inter2Pose = new Pose(27.2,121.7,Math.toRadians(315));
    final Pose scorePose = new Pose(15, 130, Math.toRadians(315));
    final Pose scorePose2 = new Pose(16,130.5, Math.toRadians(315));
    final Pose pickup1Pose = new Pose(31.2, 121.5, Math.toRadians(0));
    final Pose pickup2Pose = new Pose(31.2,130,Math.toRadians(0)); //41 19
    final Pose pickup3Pose = new Pose(35.6,12,Math.toRadians(300));

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

        pickup1 = new Path(new BezierLine(new Point(preloadPose), new Point(pickup1Pose)));
        pickup1.setLinearHeadingInterpolation(preloadPose.getHeading(), pickup1Pose.getHeading());

        scorePickup1Inter = new Path(new BezierLine(new Point(pickup1Pose), new Point(interPose)));
        scorePickup1Inter.setLinearHeadingInterpolation(pickup1Pose.getHeading(), interPose.getHeading());

        scorePickup1 = new Path(new BezierLine(new Point(interPose), new Point(scorePose)));
        scorePickup1.setLinearHeadingInterpolation(interPose.getHeading(), scorePose.getHeading());
//        scorePickup1Chain = new PathChain(
//                scorePickup1Inter,
//                scorePickup1
//        );
        scorePickup1Chain = follower.pathBuilder()
                .addPath(scorePickup1Inter)
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), interPose.getHeading())
                .addPath(scorePickup1)
                .setLinearHeadingInterpolation(interPose.getHeading(), scorePose.getHeading())
                .build();

        pickup2 = new Path(new BezierLine(new Point(scorePose), new Point(pickup2Pose)));
        pickup2.setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading());

        scorePickup2Inter = new Path(new BezierLine(new Point(pickup2Pose), new Point(inter2Pose)));
        scorePickup2Inter.setLinearHeadingInterpolation(pickup2Pose.getHeading(),inter2Pose.getHeading());

        scorePickup2 = new Path(new BezierLine(new Point(inter2Pose), new Point(scorePose2)));
        scorePickup2.setLinearHeadingInterpolation(inter2Pose.getHeading(), scorePose2.getHeading());


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
                    linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.BSK_HIGH;
                    scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.SCORE;
                    status = STROBOT.PLACE_PRELOAD;
                    break;
                }
                case PLACE_PRELOAD: {
                    if(TimerPlacePreload.seconds() > 1.5)
                    {
                        claw.currentStatus= ClawController.ClawStatus.OPEN;
                    }
                    if(TimerPlacePreload.seconds() > 2)
                    {
                        follower.followPath(pickup1);
                        status=STROBOT.GRAB_PICKUP1;
                    }
                    break;
                }
                case GRAB_PICKUP1:
                {
                    if(TimerPlacePreload.seconds()>1.7)
                    {
                        linkageSlides.currentStatus = LinkageSlidesController.LinkageSlidesStatus.INIT;
                    }
                    if(TimerPlacePreload.seconds()>2)
                    {
                        slides.currentStatus= SlidesController.SlidesStatus.INIT;
                        linkage.currentStatus = LinkageController.LinkageStatus.COLLECT;
                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.COLLECT_FROM_SUB;
                    }
                    if(follower.isBusy())
                    {
                        TimerPickup.reset();
                    }
                    if(!follower.isBusy())
                    {
                        fourbar.currentStatus = FourbarController.FourbarStatus.COLLECT_SUB;
                        clawPosition.currentStatus = ClawPositionController.ClawPositionStatus.COLLECT_SUB;
                        if (TimerPickup.seconds() > 0.25) {
                            claw.currentStatus = ClawController.ClawStatus.CLOSE;
                        }
                        if(TimerPickup.seconds() > 0.50)
                        {
                            linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.INIT;
                            linkage.currentStatus= LinkageController.LinkageStatus.INIT;
                            clawRotate.currentStatus = ClawRotateController.ClawRotateStatus.INIT;
                            fourbar.currentStatus = FourbarController.FourbarStatus.INIT;
                            clawPosition.currentStatus = ClawPositionController.ClawPositionStatus.INIT;
                            TimerScore.reset();
                            follower.followPath(scorePickup1Inter);
                            status=STROBOT.SCORE_INTER1;
                        }
                    }
                    break;
                }
                case SCORE_INTER1:
                {
                    if(follower.isBusy())
                    {
                        TimerScore.reset();
                    }
                    if(!follower.isBusy())
                    {
                        if(TimerScore.seconds()>0.5)
                        {
                            follower.followPath(scorePickup1);
                            TimerScore.reset();
                            status=STROBOT.SCORE_PICKUP1;
                        }
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.BSK_HIGH;
                        scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.SCORE;
                    }
                    break;
                }
                case SCORE_PICKUP1:
                {
                    if(TimerScore.seconds()>0)
                    {
                    }
                    if(!follower.isBusy() && TimerScore.seconds()>0.7)
                    {
//                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.FLICK;
                        claw.currentStatus= ClawController.ClawStatus.OPEN;
                        if(TimerScore.seconds()>2.5)
                        {
                            follower.followPath(pickup2);
                            TimerLeave.reset();
                            status = STROBOT.GRAB_PICKUP2;
                        }
                    }
                    break;
                }
                case GRAB_PICKUP2:
                {
                    linkageSlides.currentStatus = LinkageSlidesController.LinkageSlidesStatus.INIT;
                    slides.currentStatus= SlidesController.SlidesStatus.INIT;
                    linkage.currentStatus = LinkageController.LinkageStatus.COLLECT;
                    scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.COLLECT_FROM_SUB;
                    if(follower.isBusy())
                    {
                        TimerPickup.reset();
                    }
                    if(!follower.isBusy())
                    {
                        fourbar.currentStatus = FourbarController.FourbarStatus.COLLECT_SUB;
                        clawPosition.currentStatus = ClawPositionController.ClawPositionStatus.COLLECT_SUB;
                        if (TimerPickup.seconds() > 0.25) {
                            claw.currentStatus = ClawController.ClawStatus.CLOSE;
                        }
                        if(TimerPickup.seconds() > 0.50)
                        {
                            linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.INIT;
                            linkage.currentStatus= LinkageController.LinkageStatus.INIT;
                            clawRotate.currentStatus = ClawRotateController.ClawRotateStatus.INIT;
                            fourbar.currentStatus = FourbarController.FourbarStatus.INIT;
                            clawPosition.currentStatus = ClawPositionController.ClawPositionStatus.INIT;
                            TimerScore.reset();
                            follower.followPath(scorePickup2Inter);
                            status=STROBOT.SCORE_INTER2;
                        }
                    }
                    break;
                }
                case SCORE_INTER2:
                {
                    if(follower.isBusy())
                    {
                        TimerScore.reset();
                    }
                    if(!follower.isBusy())
                    {
                        if(TimerScore.seconds()>0.5)
                        {
                            follower.followPath(scorePickup2);
                            TimerScore.reset();
                            status=STROBOT.SCORE_PICKUP2;
                        }
                        linkageSlides.currentStatus= LinkageSlidesController.LinkageSlidesStatus.BSK_HIGH;
                        scoreSystem.currentStatus = ScoreSystemController.ScoreSystemStatus.SCORE;
                    }
                    break;
                }
                case SCORE_PICKUP2:
                {
                    if(TimerScore.seconds()>0)
                    {
                    }
                    if(!follower.isBusy() && TimerScore.seconds()>0.7)
                    {
//                        scoreSystem.currentStatus= ScoreSystemController.ScoreSystemStatus.FLICK;
                        claw.currentStatus= ClawController.ClawStatus.OPEN;
                        if(TimerScore.seconds()>2.5)
                        {
//                            follower.followPath(pickup2);
//                            TimerLeave.reset();
//                            status = STROBOT.GRAB_PICKUP2;
                        }
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