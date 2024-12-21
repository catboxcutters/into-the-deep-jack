package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ScoreSystemController
{
    public enum ScoreSystemStatus {
        INIT,
        LOWER_FOURBAR,
        GRAB_SAMPLE,
        LOWER_FOURBAR_SUB, COLLECT_FROM_SUB,
        FLICK,SCORE, OPEN_RUNG, RUNG
    }
    public ScoreSystemStatus currentStatus = ScoreSystemStatus.INIT;
    public ScoreSystemStatus previousStatus=null;
    private ClawController clawController = null;
    private ClawRotateController clawRotateController = null;
    private FourbarController fourbarController = null;
    private ClawPositionController clawPositionController = null;
    public ElapsedTime timer = new ElapsedTime();
    public ElapsedTime timer_flick = new ElapsedTime();
    public ElapsedTime timer_reset = new ElapsedTime();
    double delay=0.3;
    double delay2=0.3;
    boolean ok;
    public ScoreSystemController(ClawController clawController, ClawRotateController clawRotateController, FourbarController fourbarController, ClawPositionController clawPositionController)
    {
        this.clawController = clawController;
        this.clawRotateController = clawRotateController;
        this.fourbarController = fourbarController;
        this.clawPositionController = clawPositionController;
    }
    public void update()
    {
        if(currentStatus!=previousStatus || currentStatus==ScoreSystemStatus.GRAB_SAMPLE || currentStatus==ScoreSystemStatus.LOWER_FOURBAR_SUB || currentStatus==ScoreSystemStatus.FLICK || currentStatus==ScoreSystemStatus.RUNG || currentStatus==ScoreSystemStatus.OPEN_RUNG)
        {
            previousStatus=currentStatus;
            switch(currentStatus)
            {
                case INIT:
                {
                    clawRotateController.currentStatus = ClawRotateController.ClawRotateStatus.INIT;
                    fourbarController.currentStatus = FourbarController.FourbarStatus.INIT;
                    clawPositionController.currentStatus = ClawPositionController.ClawPositionStatus.INIT;
                    break;
                }
                case LOWER_FOURBAR:
                {
                    fourbarController.currentStatus = FourbarController.FourbarStatus.COLLECT;
                    timer.reset();
                    currentStatus= ScoreSystemStatus.GRAB_SAMPLE;
                    ok=false;
                    break;
                }
                case GRAB_SAMPLE:
                {
                    if(timer.seconds()>delay && ok==false)
                    {
                        clawController.currentStatus = ClawController.ClawStatus.CLOSE;
                        timer.reset();
                        ok=true;
                    }
                    if(timer.seconds()>delay2 && ok==true)
                    {
                        currentStatus= ScoreSystemStatus.INIT;
                    }
                    break;
                }
                case COLLECT_FROM_SUB:
                {
                    fourbarController.currentStatus = FourbarController.FourbarStatus.SUB;
                    clawPositionController.currentStatus = ClawPositionController.ClawPositionStatus.SUB;
                    break;
                }
                case LOWER_FOURBAR_SUB:
                {
                    fourbarController.currentStatus = FourbarController.FourbarStatus.COLLECT_SUB;
                    clawPositionController.currentStatus = ClawPositionController.ClawPositionStatus.COLLECT_SUB;
                    if(timer.seconds()>0.1)
                    {
                        clawController.currentStatus= ClawController.ClawStatus.CLOSE;
                    }
                    if(timer.seconds()>0.2)
                    {
                        currentStatus=ScoreSystemStatus.INIT;
                    }
                    break;
                }
                case FLICK:
                {
                    fourbarController.currentStatus= FourbarController.FourbarStatus.FLICK;
                    if(timer_flick.seconds()>0.07)
                    {
                        clawController.currentStatus= ClawController.ClawStatus.OPEN;
                    }
                    break;
                }
                case SCORE:
                {
                    fourbarController.currentStatus= FourbarController.FourbarStatus.SCORE;
                    clawPositionController.currentStatus= ClawPositionController.ClawPositionStatus.SCORE;
                    break;
                }
                case RUNG:
                {
                    fourbarController.currentStatus= FourbarController.FourbarStatus.RUNG;
                    clawPositionController.currentStatus= ClawPositionController.ClawPositionStatus.RUNG;
                    timer_reset.reset();
                    break;
                }
                case OPEN_RUNG:
                {
                    if(timer_reset.seconds()>0.2) {
                        clawController.currentStatus = ClawController.ClawStatus.OPEN;
                    }
                    }
                    if(timer_reset.seconds()>0.5) {
                        currentStatus=ScoreSystemStatus.INIT;}
                    break;
                }
            }
        }
    }

