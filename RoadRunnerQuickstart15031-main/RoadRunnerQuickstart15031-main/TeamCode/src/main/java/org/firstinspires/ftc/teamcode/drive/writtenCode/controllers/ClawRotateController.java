package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

@Config
public class ClawRotateController {
    public enum ClawRotateStatus{
        INIT,
        MINUS,MINUS2,
        PLUS,PLUS2,
        RUNTO;
    }
    public ClawRotateStatus currentStatus = ClawRotateStatus.INIT;
    public ClawRotateStatus previousStatus=null;
    public static double init_position=0.5;
    public Servo clawRotate = null;
    public ClawRotateController(RobotMap robot) {
        this.clawRotate=robot.clawRotate;
    }
    public void update(double target)
    {
        if(currentStatus!=previousStatus || currentStatus == ClawRotateStatus.RUNTO){
            previousStatus=currentStatus;
            switch(currentStatus)
            {
                case INIT:
                {
                    this.clawRotate.setPosition(init_position);
                    break;
                }
                case PLUS:
                {
                    this.clawRotate.setPosition(init_position+0.17);
                    break;
                }
                case PLUS2:
                {
                    this.clawRotate.setPosition(init_position+0.315);
                    break;
                }
                case MINUS:
                {
                    this.clawRotate.setPosition(init_position-0.17);
                    break;
                }
                case MINUS2:
                {
                    this.clawRotate.setPosition(init_position-0.315);
                    break;
                }
                case RUNTO:
                {
                    this.clawRotate.setPosition(target);
                    break;
                }
            }
        }
    }
}