package org.firstinspires.ftc.teamcode.drive.writtenCode;

import android.transition.Slide;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ClawController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ClawPositionController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ClawRotateController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.FourbarController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.LinkageController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.LinkageSlidesController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ScoreSystemController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.SlidesController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.CameraController;


@TeleOp(name="TeleOpCode", group="Linear OpMode")
public class TeleOpCode extends LinearOpMode {
    public void setMotorRunningMode(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront,
                                    DcMotor rightBack, DcMotor.RunMode runningMode) {
        leftFront.setMode(runningMode);
        rightFront.setMode(runningMode);
        leftBack.setMode(runningMode);
        rightBack.setMode(runningMode);
    }

    public void setMotorZeroPowerBehaviour(DcMotor leftFront, DcMotor leftBack, DcMotor rightFront,
                                           DcMotor rightBack, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        leftFront.setZeroPowerBehavior(zeroPowerBehavior);
        rightFront.setZeroPowerBehavior(zeroPowerBehavior);
        leftBack.setZeroPowerBehavior(zeroPowerBehavior);
        rightBack.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void robotCentricDrive(DcMotor leftFront, DcMotor leftBack,
                                  DcMotor rightFront, DcMotor rightBack,
                                  double leftTrigger, double rightTrigger, double rate) {

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = (-gamepad1.left_trigger + gamepad1.right_trigger) * 1.05; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;


        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator *rate;
        double leftBackPower = (y - x + rx) / denominator *rate;
        double rightFrontPower = (y - x - rx) / denominator *rate;
        double rightBackPower = (y + x - rx) / denominator *rate;


        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }

    ElapsedTime GlobalTimer = new ElapsedTime();
    ElapsedTime BeamTimer = new ElapsedTime();
    ElapsedTime SlidesTimer = new ElapsedTime();
    ElapsedTime timer_release = new ElapsedTime();

    double time_to_slides = 0.35;
    double time_to_start_beam = 0.15;
    int linkage_target_position = 0;
    int slides_target_position = 0;
    double claw_rotate_target =0;
    public double rate=1;
    boolean ok=false;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        RobotMap robot= new RobotMap(hardwareMap);

        LinkageController linkageController = new LinkageController(robot);
        SlidesController slidesController = new SlidesController(robot);
        FourbarController fourbarController = new FourbarController(robot);
        ClawRotateController clawRotateController = new ClawRotateController(robot);
        ClawController clawController = new ClawController(robot);
        ClawPositionController clawPositionController = new ClawPositionController(robot);
        ScoreSystemController scoreSystemController = new ScoreSystemController(clawController, clawRotateController,fourbarController,clawPositionController);
        LinkageSlidesController linkageSlidesController = new LinkageSlidesController(linkageController, slidesController);
        DigitalChannel beam = robot.beam;
        CameraController cameraController = new CameraController(hardwareMap, "Webcam 1");
        linkageController.update(LinkageController.init_position,linkage_target_position);
        slidesController.update(SlidesController.init_position,slides_target_position);

        clawRotateController.update(ClawRotateController.init_position);
        fourbarController.update();
        clawPositionController.update();
        clawController.update();
        scoreSystemController.update();
        linkageSlidesController.update();

        DcMotor rightFront = hardwareMap.get(DcMotor.class,"rightFront");
        DcMotor leftFront = hardwareMap.get(DcMotor.class,"leftFront");
        DcMotor rightBack = hardwareMap.get(DcMotor.class,"rightBack");
        DcMotor leftBack = hardwareMap.get(DcMotor.class,"leftBack");
        MotorConfigurationType mct1, mct2, mct3, mct4;
        mct1 = rightBack.getMotorType().clone();
        mct1.setAchieveableMaxRPMFraction(1.0);
        rightBack.setMotorType(mct1);

        mct2 = rightFront.getMotorType().clone();
        mct2.setAchieveableMaxRPMFraction(1.0);
        rightFront.setMotorType(mct2);

        mct3 = leftFront.getMotorType().clone();
        mct3.setAchieveableMaxRPMFraction(1.0);
        leftFront.setMotorType(mct3);

        mct4 = leftBack.getMotorType().clone();
        mct4.setAchieveableMaxRPMFraction(1.0);
        leftBack.setMotorType(mct4);
        setMotorRunningMode(leftFront,leftBack,rightFront,rightBack,
                DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        setMotorZeroPowerBehaviour(leftFront,leftBack,rightFront,rightBack,
                DcMotor.ZeroPowerBehavior.BRAKE);
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        cameraController.startCamera();
        waitForStart();
        GlobalTimer.reset();
        while(opModeIsActive())
        {
            if(isStopRequested()) return;
            int slides_current_position = robot.linkage.getCurrentPosition();
            int linkage_current_position = robot.encoderLinkage.getCurrentPosition();
            robotCentricDrive(leftFront,leftBack,rightFront,rightBack,gamepad1.left_trigger,gamepad1.right_trigger, rate);
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
            if(currentGamepad1.right_bumper && !previousGamepad1.right_bumper)
            {
                switch(linkageSlidesController.currentStatus){
                    case INIT:
                        clawController.currentStatus = ClawController.ClawStatus.OPEN;
                        linkageSlidesController.currentStatus = LinkageSlidesController.LinkageSlidesStatus.LOWER_LINKAGE;
                        scoreSystemController.currentStatus= ScoreSystemController.ScoreSystemStatus.COLLECT_FROM_SUB;
                        rate = 0.4;
                        break;
                    case EXTEND_SLIDES:
                        scoreSystemController.timer.reset();
                        SlidesTimer.reset();
                        ok=true;
                        scoreSystemController.currentStatus = ScoreSystemController.ScoreSystemStatus.LOWER_FOURBAR_SUB;
                        rate = 1;
                        break;
                }
            }
            if(SlidesTimer.seconds()>time_to_slides && ok==true)
            {
                linkageSlidesController.currentStatus = LinkageSlidesController.LinkageSlidesStatus.INIT;
                ok=false;
            }
            double test = 1;
            if(currentGamepad2.y && !previousGamepad2.y)
            {
                switch(scoreSystemController.currentStatus){
                    case INIT:
                        scoreSystemController.currentStatus = ScoreSystemController.ScoreSystemStatus.LOWER_FOURBAR;
                        break;
                }
            }
            if(currentGamepad2.a && !previousGamepad2.a)
            {
                switch(linkageSlidesController.currentStatus)
                {
                    case INIT:
                        linkageSlidesController.currentStatus= LinkageSlidesController.LinkageSlidesStatus.BSK_HIGH;
                        scoreSystemController.currentStatus= ScoreSystemController.ScoreSystemStatus.SCORE;
                        break;
                    default:
                        linkageSlidesController.currentStatus= LinkageSlidesController.LinkageSlidesStatus.INIT;
                        scoreSystemController.currentStatus= ScoreSystemController.ScoreSystemStatus.INIT;
                        clawPositionController.currentStatus= ClawPositionController.ClawPositionStatus.INIT;
                        break;
                }
            }
            if(currentGamepad2.b && !previousGamepad2.b)
            {
                switch(linkageSlidesController.currentStatus)
                {
                    case INIT:
                        linkageSlidesController.currentStatus= LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG;
                        scoreSystemController.currentStatus= ScoreSystemController.ScoreSystemStatus.RUNG;
                        break;
                    case HIGH_RUNG:
                        linkageSlidesController.currentStatus= LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE;
                        scoreSystemController.currentStatus= ScoreSystemController.ScoreSystemStatus.OPEN_RUNG;
                        timer_release.reset();
                        break;
                }
            }
            if(timer_release.seconds()>0.4 && linkageSlidesController.currentStatus== LinkageSlidesController.LinkageSlidesStatus.HIGH_RUNG_SCORE) {
                linkageSlidesController.currentStatus= LinkageSlidesController.LinkageSlidesStatus.INIT;
            }


                if(beam.getState()==true)
            {
                BeamTimer.reset();
            }
            else if(BeamTimer.seconds()>time_to_start_beam && scoreSystemController.currentStatus == ScoreSystemController.ScoreSystemStatus.INIT)
            {
                scoreSystemController.currentStatus = ScoreSystemController.ScoreSystemStatus.LOWER_FOURBAR;
                BeamTimer.reset();
            }
            if (gamepad2.left_stick_y <0 || gamepad1.right_stick_y<-0.7 && linkageSlidesController.currentStatus == LinkageSlidesController.LinkageSlidesStatus.EXTEND_SLIDES)
            {
                int manual_rate = 5000;
                if(gamepad1.right_stick_y<-0.7 && linkageSlidesController.currentStatus == LinkageSlidesController.LinkageSlidesStatus.EXTEND_SLIDES) manual_rate = 2000;
                slidesController.currentStatus = SlidesController.SlidesStatus.RUNTO;
                slides_target_position =
                        Math.max(-52000, slides_current_position-manual_rate);
            }
            else
            if (gamepad2.left_stick_y >0 || (gamepad1.right_stick_y>0.7 && linkageSlidesController.currentStatus == LinkageSlidesController.LinkageSlidesStatus.EXTEND_SLIDES))
            {
                /// Ii dau clip la currentPosition intre initPosition si highPosition.
                int manual_rate = 5000;
                if(gamepad1.right_stick_y>0.7 && linkageSlidesController.currentStatus == LinkageSlidesController.LinkageSlidesStatus.EXTEND_SLIDES) manual_rate = 2000;
                slidesController.currentStatus = SlidesController.SlidesStatus.RUNTO;
                slides_target_position =
                        Math.max(-52000, slides_current_position+manual_rate);  //Math.max(slides_current_position+10,//era +10, dar mergea invers 0)
            }
            else
            {
                slides_target_position = slides_current_position;
            }
            if(currentGamepad2.dpad_right && !previousGamepad2.dpad_right)
            {
                switch(clawRotateController.currentStatus)
                {
                    case INIT:
                        clawRotateController.currentStatus = ClawRotateController.ClawRotateStatus.MINUS;
                        break;
                    case MINUS:
                        clawRotateController.currentStatus = ClawRotateController.ClawRotateStatus.MINUS2;
                        break;
                    case PLUS:
                        clawRotateController.currentStatus = ClawRotateController.ClawRotateStatus.INIT;
                        break;
                    case PLUS2:
                        clawRotateController.currentStatus = ClawRotateController.ClawRotateStatus.PLUS;
                        break;
                }
            }
            if(currentGamepad2.dpad_left && !previousGamepad2.dpad_left)
            {
                switch(clawRotateController.currentStatus)
                {
                    case INIT:
                        clawRotateController.currentStatus = ClawRotateController.ClawRotateStatus.PLUS;
                        break;
                    case PLUS:
                        clawRotateController.currentStatus = ClawRotateController.ClawRotateStatus.PLUS2;
                        break;
                    case MINUS:
                        clawRotateController.currentStatus = ClawRotateController.ClawRotateStatus.INIT;
                        break;
                    case MINUS2:
                        clawRotateController.currentStatus = ClawRotateController.ClawRotateStatus.MINUS;
                        break;
                }
            }
            //asta trb sa fie mereu la final
            if(currentGamepad2.right_stick_button && !previousGamepad2.right_stick_button)
            {
               switch (linkageSlidesController.currentStatus) {
                   default:
                       switch (clawController.currentStatus) {
                        case CLOSE:
                               clawController.currentStatus = ClawController.ClawStatus.OPEN;
                               break;
                        case OPEN:
                               clawController.currentStatus = ClawController.ClawStatus.CLOSE;
                               break;
                       }
                        break;
                   case SCORE:
                   case BSK_HIGH:
                   case BSK_LOW:
                       switch(scoreSystemController.currentStatus)
                          {
                              case SCORE:
                                 scoreSystemController.timer_flick.reset();
                                 scoreSystemController.currentStatus = ScoreSystemController.ScoreSystemStatus.FLICK;
                                 break;
                                case FLICK:
                                 scoreSystemController.currentStatus= ScoreSystemController.ScoreSystemStatus.SCORE;
                                 break;
                          }
               }
            }
            linkageController.update(linkage_current_position,linkage_target_position);
            slidesController.update(slides_current_position,slides_target_position);
            clawController.update();
            fourbarController.update();
            clawPositionController.update();
            clawRotateController.update(claw_rotate_target);
            scoreSystemController.update();
            linkageSlidesController.update();

            telemetry.addData("fourbar status", fourbarController.currentStatus);
            telemetry.addData("claw status", clawController.currentStatus);
            telemetry.addData("beam", beam.getState());
            telemetry.addData("score system", scoreSystemController.currentStatus);
            telemetry.addData("linkageslides system", linkageSlidesController.currentStatus);
            telemetry.addData("ok", ok);
            telemetry.addData("slides", slidesController.currentStatus);
            telemetry.addData("timer_release", timer_release.seconds());
            telemetry.update();
        }
    }
}