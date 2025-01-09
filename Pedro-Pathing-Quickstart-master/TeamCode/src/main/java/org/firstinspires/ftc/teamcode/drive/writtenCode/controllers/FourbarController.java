package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

@Config
public class FourbarController {
    public enum FourbarStatus{
        INIT,
        COLLECT,
        SUB,
        COLLECT_SUB,
        RUNTO,
        FLICK,
        RUNG,
        SCORE,
        SCORE_LOW,
        FEED, RUNG_PARA;
    }
    public FourbarStatus currentStatus = FourbarStatus.INIT;
    public FourbarStatus previousStatus=null;
    public static double init_position=0.8;
    public static double rung_position = 0.25;
    public static double sub_position = 0.42;
    public static double sub_collect_position = 0.3;
    public static double collect_position = 0.86;
    public static double score_position = 0.43;
    public static double score_low = 0.6;
    public static double rung_para_position = 0.05;
    public static double flick = 0.47;
    public static double feed = 0.7;
    public Servo fourbarLeft = null, fourbarRight = null;
    public FourbarController(RobotMap robot) {
        this.fourbarLeft=robot.fourbarLeft;
        this.fourbarRight = robot.fourbarRight;
    }
    public void update()
    {
        if(currentStatus!=previousStatus || currentStatus == FourbarStatus.RUNTO){
            previousStatus=currentStatus;
            switch(currentStatus)
            {
                case INIT:
                {
                    this.fourbarLeft.setPosition(init_position);
                    this.fourbarRight.setPosition(init_position);
                    break;
                }
                case COLLECT:
                {
                    this.fourbarLeft.setPosition(collect_position);
                    this.fourbarRight.setPosition(collect_position);
                    break;
                }
                case COLLECT_SUB:
                {
                    this.fourbarLeft.setPosition(sub_collect_position);
                    this.fourbarRight.setPosition(sub_collect_position);
                    break;
                }
                case SUB:
                {
                    this.fourbarLeft.setPosition(sub_position);
                    this.fourbarRight.setPosition(sub_position);
                    break;
                }
                case SCORE:
                {
                    this.fourbarLeft.setPosition(score_position);
                    this.fourbarRight.setPosition(score_position);
                    break;
                }
                case SCORE_LOW:
                {
                    this.fourbarLeft.setPosition(score_low);
                    this.fourbarRight.setPosition(score_low);
                    break;
                }
                case FLICK:
                {
                    this.fourbarLeft.setPosition(flick);
                    this.fourbarRight.setPosition(flick);
                    break;
                }
                case FEED:
                {
                    this.fourbarLeft.setPosition(feed);
                    this.fourbarRight.setPosition(feed);
                    break;
                }
                case RUNG:
                {
                    this.fourbarLeft.setPosition(rung_position);
                    this.fourbarRight.setPosition(rung_position);
                    break;
                }
                case RUNG_PARA:
                {
                    this.fourbarLeft.setPosition(rung_para_position);
                    this.fourbarRight.setPosition(rung_para_position);
                    break;
                }
            }
        }
    }
}