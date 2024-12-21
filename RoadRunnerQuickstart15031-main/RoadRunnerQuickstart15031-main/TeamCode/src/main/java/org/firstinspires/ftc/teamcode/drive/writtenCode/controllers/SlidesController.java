package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import android.widget.SimpleExpandableListAdapter;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

public class SlidesController {
    public enum SlidesStatus {
        INIT,
        COLLECT,
        BSK_LOW,
        BSK_MID,
        BSK_HIGH,
        RUNTO,
        LOW_RUNG,
        LOW_RUNG_SCORE,
        HIGH_RUNG,
        HIGH_RUNG_SCORE
    }
    public SlidesStatus currentStatus = SlidesStatus.INIT;
    public SlidesStatus previousStatus = null;
    public static int init_position = 0;
    public static int collect_position = -22000;
    public static int bsk_low_position = -600;
    public static int bsk_mid_position = -900;
    public static int bsk_high_position = -51500;
    public static int low_rung = -7000;
    public static int high_rung = -27000;
    public static int low_rung_score = -6000;
    public static int high_rung_score = -19000;
    public int current_position = init_position;
    public DcMotorEx slidesLeft = null;
    public DcMotorEx slidesMid = null;
    public DcMotorEx slidesRight = null;
    public DcMotorEx encoderSlides = null;
    SimplePIDController slidesPID = null;
    SimplePIDController slidesPID_horizontal=null;
    SimplePIDController PID = null;

    public static double KpH = 0.00025;//0.00325
    public static double KiH = 0.0001;//0.0022
    public static double KdH = 0.001;
    public static double Kp = 0.00025;//0.00325
    public static double Ki = 0.0001;//0.0022
    public static double Kd = 0.001; //0.001
    public static double PowerCap = 1;
    public static double maxSpeed = 1;


    public SlidesController(RobotMap robot) {
        this.slidesLeft = robot.slidesLeft;
        this.slidesMid = robot.slidesMid;
        this.slidesRight = robot.slidesRight;
        this.encoderSlides = robot.linkage;
        slidesPID = new SimplePIDController(Kp, Ki, Kd);
        slidesPID.targetValue = init_position;
        slidesPID.maxOutput = maxSpeed;
        slidesPID_horizontal = new SimplePIDController(KpH, KiH, KdH);
        slidesPID_horizontal.targetValue = init_position;
        slidesPID_horizontal.maxOutput = maxSpeed;
        PID=slidesPID;
    }
    public void update(int slides_position, int runto_target)
    {
        double powerColectare = PID.update(encoderSlides.getCurrentPosition());
        powerColectare = Math.max(-PowerCap,Math.min(powerColectare,PowerCap));
        this.slidesLeft.setPower(-powerColectare);
        this.slidesRight.setPower(-powerColectare);
        this.slidesMid.setPower(-powerColectare);

        double slides_current_position = encoderSlides.getCurrentPosition();
        if(currentStatus!=previousStatus || currentStatus==SlidesStatus.RUNTO){
            previousStatus=currentStatus;
            switch(currentStatus) {
                case INIT:
                {
                    PID=slidesPID;
                    PID.targetValue = init_position;
                    break;
                }
                case COLLECT:
                {
                    PID=slidesPID_horizontal;
                    PID.targetValue = collect_position;
                    break;
                }
                case BSK_LOW:
                {
                    PID=slidesPID;
                    PID.targetValue = bsk_low_position;
                    break;
                }
                case BSK_MID:
                {
                    PID=slidesPID;
                    PID.targetValue = bsk_mid_position;
                    break;
                }
                case BSK_HIGH:
                {
                    PID=slidesPID;
                    PID.targetValue = bsk_high_position;
                    break;
                }
                case RUNTO:
                {
                    PID=slidesPID;
                    PID.targetValue = runto_target;
                    break;
                }
                case LOW_RUNG:
                {
                    PID=slidesPID;
                    PID.targetValue = low_rung;
                    break;
                }
                case LOW_RUNG_SCORE:
                {
                    PID=slidesPID;
                    PID.targetValue = low_rung_score;
                    break;
                }
                case HIGH_RUNG:
                {
                    PID=slidesPID;
                    PID.targetValue = high_rung;
                    break;
                }
                case HIGH_RUNG_SCORE:
                {
                    PID=slidesPID;
                    PID.targetValue = high_rung_score;
                    break;
                }
            }
        }
        if (Kp!=PID.p || Kd!=PID.d || Ki!=PID.i || maxSpeed !=PID.maxOutput )
        {
            PID.p = Kp;
            PID.d = Kd;
            PID.i = Ki;
            PID.maxOutput = maxSpeed;
        }
    }
}
