package Util;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.sql.Time;

@Config
public class CliprackSubsystem extends SubsystemBase {

    //TODO: alter the up and down positions for cliprack

    //Servo for left of cliprack
    private static Servo leftServo;
    private double leftUp = .2;
    private double leftDown = 1;
    public static double leftTarget = 0;


    //Servo for right of clip rack
    private static Servo rightServo;
    private double rightUp = .3;
    private double rightDown = 0;
    public static double rightTarget = 0;

    //servo for rotating the clips
    private static Servo clipRotationServo;
    public static double rotationStart = 1;
    public static double rotationReady = .55;
    public static double rotationStop = .1;
    public static double rotationTarget = .35;

    //servo for rotating the clips
    private static Servo clipArmServo;
    private double clipArmDown = 0;
    private double clipArmUp = 0.35;
    public static double clipArmTarget = 0;


    public CliprackSubsystem(final HardwareMap hMap){

        leftServo = hMap.get(Servo.class, "LCS");
        rightServo = hMap.get(Servo.class, "RCS");
        clipRotationServo = hMap.get(Servo.class, "CRS");
        clipArmServo = hMap.get(Servo.class, "CARM");

    }

    public void rackUp(){
        leftTarget = leftUp;
        rightTarget = rightUp;
        rotationTarget = .6;
        clipArmDown();

    }

    public void rackMiddle(){
        leftTarget = .6;
        rightTarget = .15;

    }

    public void rackDown(){
        leftTarget = leftDown;
        rightTarget = rightDown;
        clipArmUp();
    }

    public void raiseRight(){
        leftTarget = .4;
        rightTarget = rightUp;
    }

    public void raiseLeft(){
        leftTarget = leftUp;
        rightTarget = rightDown;
    }

    public void rotateClipStart(){
        rotationTarget = rotationStart;
    }
    public void rotateClipReady(){
        rotationTarget = rotationReady;
    }
    public void rotateClipStop(){
        rotationTarget = rotationStop;
    }

    public void clipArmUp(){
        clipArmTarget = clipArmUp;
    }
    public void clipArmDown(){
        clipArmTarget = clipArmDown;
    }

    public void setSetUp(){
        rackDown();
        rotateClipStart();
        clipArmUp();
    }

    public void readyClip(){
        rackDown();
        rotateClipStart();
    }



    public void update(){

        leftServo.setPosition(leftTarget);
        rightServo.setPosition(rightTarget);
        clipRotationServo.setPosition(rotationTarget);
        clipArmServo.setPosition(clipArmTarget);

    }

    public void sendTelemetry(Telemetry tele){
        tele.addData("Left Servo Target ", leftTarget);
        tele.addData("Right Servo Target ", rightTarget);
        tele.addData("Clip Rotation Servo Target ", rotationTarget);
        tele.addData("Clip Arm Servo Target ", clipArmTarget);

    }


}
