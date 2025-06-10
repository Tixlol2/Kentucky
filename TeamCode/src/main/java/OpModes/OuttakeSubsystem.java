package OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Util.PDFLController;

@Config
public class OuttakeSubsystem extends SubsystemBase {

    //Servo for open and close
    private static Servo grabber;
    private double grabberOpen = 0.4;
    private double grabberClose = 0.5;
    public static double grabTarget = 0;


    //Servo for horizontal rotation
    private static Servo horizontal;
    private double horizontalPerp = 0.35;
    private double horizontalPara = 0.7;
    public static double horizontalTarget = 0;


    //Servo for vertical rotation
    private static Servo vertical;
    private double verticalDown = 1;
    private double verticalUp = .35;
    public static double verticalTarget = 0;



    DcMotorEx heightMotor;
    public static int heightExtensionTarget = 0;
    public static int heightTargetMax = 1700;

    //PDFL for arm raising
    PDFLController heightController;
    public static double pH = 0.01, dH, fH = -0.1, lH = 0.45;


    //Arm on the outtake rotation
    DcMotorEx rotationMotor;
    public static int rotationTarget = 0;
    public static int rotationTargetMax = 0;

    private static double pR = 0.0, dR, fR = 0, lR = 0;
    public static double pRmax = 0.0, dRmax, fRmax = 0.15, lRmax = 0.25;

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
        grabTarget = grabberClose;
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

    public void setHeightExtensionTarget(int num){
        heightExtensionTarget = num;
    }




    public void update(){

        grabber.setPosition(grabTarget);
        horizontal.setPosition(horizontalTarget);
        vertical.setPosition(verticalTarget);

        heightController.setPDFL(pH, dH, fH, lH);
        heightController.update(heightMotor.getCurrentPosition());
        heightController.setTarget(Math.min(heightExtensionTarget, heightTargetMax));
        heightMotor.setPower(heightController.runPDFL(15));


        rotationalAngle = rotationMotor.getCurrentPosition() / ((double) 1260 / 360);


        pR = pRmax  * Math.sin(Math.toRadians(rotationalAngle));
        dR = dRmax  * Math.sin(Math.toRadians(rotationalAngle));
        fR = fRmax * Math.sin(Math.toRadians(rotationalAngle));
        lR = lRmax * Math.sin(Math.toRadians(rotationalAngle));

        rotationalController.setPDFL(pR, dR, fR, lR);
        rotationalController.update(rotationMotor.getCurrentPosition());
        rotationalController.setTarget(Math.min(rotationTargetMax, rotationTarget));
        rotationMotor.setPower(rotationalController.runPDFL(15));

    }

    public void sendTelemetry(Telemetry tele){
        tele.addData("Outtake Grab Target ", grabTarget);
        tele.addData("Outtake Horizontal Rotation Target ", horizontalTarget);
        tele.addData("Outtake Vertical Rotation Target ", verticalTarget);
        tele.addLine();
        tele.addData("Outtake Extension Target ", heightExtensionTarget);
        tele.addData("Outtake Extension Reading ", heightMotor.getCurrentPosition());
        tele.addData("Outtake Extension Power Supplied ", heightController.runPDFL(15));
        tele.addLine();
        tele.addData("Outtake Rotational Target ", rotationTarget);
        tele.addData("Outtake Rotational Reading ", rotationMotor.getCurrentPosition());
        tele.addData("Outtake Rotational Angle Degrees ", rotationalAngle);
        tele.addData("Outtake Rotational Power Supplied ", rotationalController.runPDFL(15));


    }


}
