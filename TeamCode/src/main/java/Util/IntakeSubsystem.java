package Util;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class IntakeSubsystem extends SubsystemBase {

    //Servo for open and close
    private static Servo grabber;
    private double grabberOpen = 0;
    private double grabberClose = .71;
    private double grabberCloseHard = .9;
    public static double grabTarget = 0;


    //Servo for horizontal rotation
    private static Servo horizontal;
    public static double horizontalPerp = 0.6;
    public static double horizontalPara = 0.95;
    public static double horizontalTarget = 0;


    //Servo for vertical rotation
    private static Servo vertical;
    private double verticalDown = 1;
    public static double verticalTransfer = .64;
    private double verticalUp = .35;
    public static double verticalTarget = 0;


    //Servo for extension of the arm via linkage
    private static Servo extension;
    public static double extensionTarget = 0;

    //Telemetry for telemetry calls

    public IntakeSubsystem(final HardwareMap hMap){

        grabber = hMap.get(Servo.class, "GRAB");
        horizontal = hMap.get(Servo.class, "WIS");
        vertical = hMap.get(Servo.class, "VIS");
        extension = hMap.get(Servo.class, "LINK");

    }

    public void clawOpen(){
        grabTarget = grabberOpen;
    }

    public void clawClose(){
        grabTarget = grabberClose;
    }
    public void clawCloseHard(){
        grabTarget = grabberCloseHard;
    }

    public void horizontalPerpendicular(){
        horizontalTarget = horizontalPerp;
    }

    public void horizontalPara(){
        horizontalTarget = horizontalPara;
    }

    public void verticalDown(){
        verticalTarget = verticalDown;
    }

    public void verticalUp(){
        verticalTarget = verticalUp;
    }
    public void verticalTransfer(){
        verticalTarget = verticalTransfer;
    }

    public void setExtensionTarget(double target){
        extensionTarget = target;
    }

    public void setSetUp(){
        clawClose();
        horizontalPara();
        verticalUp();
        setExtensionTarget(0);
    }

    public void setCustomHorizontal(double target){
        horizontalTarget = target;
    }

    public void prepIntake(){
        clawOpen();
        verticalDown();
        horizontalPara();
    }


    public void update(){

        grabber.setPosition(grabTarget);
        horizontal.setPosition(horizontalTarget);
        vertical.setPosition(verticalTarget);
        extension.setPosition(Math.min(.8, Math.max(0, extensionTarget)));

    }

    public void sendTelemetry(Telemetry tele){
        tele.addData("Intake Grab Target ", grabTarget);
        tele.addData("Intake Horizontal Rotation Target ", horizontalTarget);
        tele.addData("Intake Vertical Rotation Target ", verticalTarget);
        tele.addData("Intake Extension Target ", extensionTarget);
    }


}
