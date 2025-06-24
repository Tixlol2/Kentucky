package Util;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class OuttakeSubsystem extends SubsystemBase {

    //Servo for open and close
    private static Servo grabber;
    private double grabberOpen = 0.4;
    private double grabberCloseSoft = 0.5;
    private double grabberCloseHard = 0.55;
    public static double grabTarget = 0;


    //Servo for horizontal rotation
    private static Servo horizontal;
    public static double horizontalTransfer = .28;

    private double horizontalPara = 0.7;
    public static double horizontalTarget = 0;


    //Servo for vertical rotation
    private static Servo vertical;
    private double verticalDown = 0;
    private double verticalUp = .35;
    public static double verticalTarget = 0;



    DcMotorEx heightMotor;
    public static int heightExtensionTarget = 0;
    public static int heightTargetMax = 1700;

    //PDFL for arm raising
    PDFLController heightController;
    public static double pH = 0.01, dH, fH = .1, lH = 0.45;


    //Arm on the outtake rotation
    DcMotorEx rotationMotor;
    public static int rotationTarget = 0;
    public static int rotationTargetMax = 1000;

    public static double pR = 0.001, dR, fR, lR = 0.2;
    public static double fRmax = 0.2;

    private static PDFLController rotationalController;
    public static double rotationPower = 0;

    private double rotationalAngle = 0;


    public OuttakeSubsystem(final HardwareMap hMap){

        grabber = hMap.get(Servo.class, "OGS");
        horizontal = hMap.get(Servo.class, "OTS");
        vertical = hMap.get(Servo.class, "OHR");
        heightMotor = hMap.get(DcMotorEx.class, "OUT");
        rotationMotor = hMap.get(DcMotorEx.class, "OAM");
        heightController = new PDFLController(pH, dH, fH, lH);
        rotationalController = new PDFLController(pR, dR, fR, lR);

    }

    public void clawOpen(){
        grabTarget = grabberOpen;
    }

    public void clawClose(){
        grabTarget = grabberCloseSoft;
    }

    public void clawCloseHard(){
        grabTarget = grabberCloseHard;
    }

    public void horizontalTransfer(){
        horizontalTarget = horizontalTransfer;
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

    public void setHeightExtensionTarget(int num){
        heightExtensionTarget = num;
    }

    public void resetRotationMotor(){
        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetHeightMotor(){
        heightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        heightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setRotationTarget(double degree){
        rotationTarget = (int) (degree * (1260/360));
    }

    public void setSetUp(){
        resetRotationMotor();
        resetHeightMotor();
        heightExtensionTarget = 300;
        grabTarget = grabberOpen;
        horizontalTarget = horizontalTransfer;
        verticalTarget = verticalDown;
        rotationTarget = 0;
    }




    public void update(){

        grabber.setPosition(grabTarget);
        horizontal.setPosition(horizontalTarget);
        vertical.setPosition(verticalTarget);

        heightController.setPDFL(pH, dH, fH, lH);
        heightController.update(heightMotor.getCurrentPosition());
        heightController.setTarget(Math.min(heightExtensionTarget, heightTargetMax));
        heightMotor.setPower(-heightController.runPDFL(10));


        rotationalAngle = rotationMotor.getCurrentPosition() / ((double) 1260 / 360);
        fR = fRmax * Math.cos(Math.toRadians(rotationalAngle) - Math.PI/2);

        rotationalController.setPDFL(pR, dR, fR, lR);
        rotationalController.update(rotationMotor.getCurrentPosition());
        rotationalController.setTarget(Math.min(rotationTargetMax, rotationTarget));
        rotationMotor.setPower(rotationalController.runPDFL(10));

    }

    public void sendTelemetry(Telemetry tele){
        tele.addData("Outtake Grab Target ", grabTarget);
        tele.addData("Outtake Horizontal Rotation Target ", horizontalTarget);
        tele.addData("Outtake Vertical Rotation Target ", verticalTarget);
        tele.addLine();
        tele.addData("Outtake Extension Target ", heightExtensionTarget);
        tele.addData("Outtake Extension Reading ", heightMotor.getCurrentPosition());
        tele.addData("Outtake Extension Power Supplied ", heightController.runPDFL(10));
        tele.addLine();
        tele.addData("Outtake Rotational Target ", rotationTarget);
        tele.addData("Outtake Rotational Reading ", rotationMotor.getCurrentPosition());
        tele.addData("Outtake Rotational Angle Degrees ", rotationalAngle);
        tele.addData("Outtake Rotational Power Supplied ", rotationalController.runPDFL(10));


    }

    public void debugTele(Telemetry tele){
        tele.addLine();
        tele.addData("fR ", fR);
        tele.addData("Direction ", rotationalController.dir);


    }


}