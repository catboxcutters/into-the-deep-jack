package org.firstinspires.ftc.teamcode.drive.writtenCode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.graphics.Camera;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class RobotMap {
    public DcMotorEx slidesLeft,slidesMid,slidesRight;
    public DcMotorEx linkage;
    public MotorConfigurationType mctIntake, mctExtenderLeft ,mctExtenderRight;

    public Servo fourbarLeft,fourbarRight;
    public Servo clawPosition, clawRotate, clawOpen;
    public ColorSensor colorSensor;
    public DigitalChannel beam;
    public Camera camera;
    public Servo PTOLeft, PTORight;
    public CRServo climbLeft, climbRight;
    public DcMotorEx encoderSlides;
    public DcMotorEx encoderLinkage;

    public RobotMap(HardwareMap Init)
    {

        slidesLeft = Init.get(DcMotorEx.class, "slidesLeft");
        slidesLeft.setDirection(DcMotor.Direction.FORWARD);
        slidesLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slidesMid = Init.get(DcMotorEx.class, "slidesMid");
        slidesMid.setDirection(DcMotor.Direction.FORWARD);
        slidesMid.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesMid.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slidesRight = Init.get(DcMotorEx.class, "slidesRight");
        slidesRight.setDirection(DcMotor.Direction.FORWARD);
        slidesRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        linkage = Init.get(DcMotorEx.class, "linkage");
        linkage.setDirection(DcMotor.Direction.FORWARD);


        //encoder for slides is linkage
        encoderSlides = Init.get(DcMotorEx.class, "linkage");
        linkage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linkage.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linkage.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        //encoder for linkage is rightback
        encoderLinkage=Init.get(DcMotorEx.class, "rightBack");
        encoderLinkage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderLinkage.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        fourbarLeft = Init.get(Servo.class, "fourbarLeft");
        fourbarRight = Init.get(Servo.class, "fourbarRight");
        clawPosition = Init.get(Servo.class, "clawPosition");
        clawRotate = Init.get(Servo.class, "clawRotate");
        clawOpen = Init.get(Servo.class, "clawOpen");

        PTOLeft = Init.get(Servo.class, "PTOLeft");
        PTORight = Init.get(Servo.class, "PTORight");

        climbLeft = Init.get(CRServo.class, "climbLeft");
        climbRight = Init.get(CRServo.class, "climbRight");

        colorSensor = Init.get(ColorSensor.class, "colorSensor");
        beam = Init.get(DigitalChannel.class, "beam");
//        camera = Init.get(Camera.class, "camera");
    }
}