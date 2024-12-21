package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import static org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.LinkageController.LinkageStatus.RUNTO;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

@Config
public class LinkageController {
    public enum LinkageStatus {
        INIT,
        COLLECT,
        RUNTO,
        LOW_RUNG,
        LOW_RUNG_SCORE,
        HIGH_RUNG,
        HIGH_RUNG_SCORE,
    }
    public LinkageStatus currentStatus = LinkageStatus.INIT;
    public LinkageStatus previousStatus = null;
    public static int init_position = 0;
    public static int collect_position = -3100;


    public DcMotorEx encoderLinkage = null;
    SimplePIDController linkagePID = null;
    public static double Kp = 0.002;//0.00325
    public static double Ki = 0;//0.0022
    public static double Kd = 0.004;
    public static double PowerCap = 1;
    public static double maxSpeed = 1;
    public int current_position = init_position;
    public DcMotorEx linkage = null;
    public LinkageController(RobotMap robot) {
        this.linkage = robot.linkage;
        this.encoderLinkage = robot.encoderLinkage;
        linkagePID = new SimplePIDController(Kp, Ki, Kd);
        linkagePID.targetValue = init_position;
        linkagePID.maxOutput = maxSpeed;

    }
    public void update(int linkage_position, int runto_target)
    {
        double powerColectare = linkagePID.update(linkage_position);
        powerColectare = Math.max(-PowerCap,Math.min(powerColectare,PowerCap));
        this.linkage.setPower(powerColectare);
        double linkage_current_position = encoderLinkage.getCurrentPosition();
        if(currentStatus!=previousStatus || currentStatus==RUNTO){
            previousStatus=currentStatus;
            switch(currentStatus) {
                case INIT:
                {
                    linkagePID.targetValue = init_position;
                    break;
                }
                case COLLECT:
                {
                    linkagePID.targetValue = collect_position;
                    break;
                }
                case RUNTO:
                {
                    linkagePID.targetValue = runto_target;
                    break;
                }

            }
        }
    }
}
