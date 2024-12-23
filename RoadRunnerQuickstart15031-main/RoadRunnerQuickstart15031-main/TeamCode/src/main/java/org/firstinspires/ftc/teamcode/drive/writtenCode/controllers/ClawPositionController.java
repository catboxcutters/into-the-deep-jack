package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

@Config
public class ClawPositionController {
    public enum ClawPositionStatus{
        INIT,
        COLLECT,
        SUB,
        COLLECT_SUB,
        SCORE,
        RUNTO,
        RUNG,
        RUNG_SIDE_RETRACT;
    }
    public ClawPositionStatus currentStatus = ClawPositionStatus.INIT;
    public ClawPositionStatus previousStatus=null;
    public static double init_position=0.7;
    public static double collect_position = 0.54;
    public static double sub_position = 0;
    public static double collect_sub_position = 0.094;
    public  static  double rung_position = 0.12;
    public static  double rung_side_retract = 0.35;
    public static double score = 0.39;
    public Servo clawPosition = null;
    public ClawPositionController(RobotMap robot) {
        this.clawPosition=robot.clawPosition;
    }
    public void update()
    {
        if(currentStatus!=previousStatus || currentStatus == ClawPositionStatus.RUNTO){
            previousStatus=currentStatus;
            switch(currentStatus)
            {
                case INIT:
                {
                    this.clawPosition.setPosition(init_position);
                    break;
                }
                case COLLECT:
                {
                    this.clawPosition.setPosition(collect_position);
                    break;
                }
                case SUB:
                {
                    this.clawPosition.setPosition(sub_position);
                    break;
                }
                case COLLECT_SUB:
                {
                    this.clawPosition.setPosition(collect_sub_position);
                    break;
                }
                case SCORE:
                {
                    this.clawPosition.setPosition(score);
                    break;
                }
                case RUNG:
                {
                    this.clawPosition.setPosition(rung_position);
                    break;
                }
                case RUNG_SIDE_RETRACT:
                {
                    this.clawPosition.setPosition(rung_side_retract);
                    break;
                }
            }
        }
    }
}