package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

@Config
public class ClawController {
    public enum ClawStatus{
        OPEN,
        CLOSE;
    }
    public ClawStatus currentStatus = ClawStatus.OPEN;
    public ClawStatus previousStatus=null;

    public static double claw_open=1;
    public static double claw_closed = 0.75;
    public Servo claw = null;


    public ClawController(RobotMap robot){
        this.claw = robot.clawOpen;
    }
    public void update(){
        if(currentStatus!=previousStatus)
        {
            previousStatus=currentStatus;
            switch(currentStatus)
            {
                case OPEN:
                {
                    this.claw.setPosition(claw_open);
                    break;
                }
                case CLOSE:
                {
                    this.claw.setPosition(claw_closed);
                    break;
                }
            }
        }
    }
}