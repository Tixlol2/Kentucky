package OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Util.CliprackSubsystem;
import Util.IntakeSubsystem;
import Util.OuttakeSubsystem;
import Util.Timer;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@TeleOp(name = "Drive", group = "freak")
public class Teleop extends OpMode {

    Follower follower;
    IntakeSubsystem intake;
    private double intakeExtensionTarget = 0;
    private double outtakeHeighthTarget = 0;
    private double rotationalTarget = 0;

    CliprackSubsystem cliprack;

    OuttakeSubsystem outtake;

    private int transferCase = 0;

    Timer timer;

    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(Auton.startPose);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intake = new IntakeSubsystem(hardwareMap);
        cliprack = new CliprackSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        intake.clawClose();
        intake.horizontalPara();
        intake.verticalUp();

        cliprack.rackDown();
        cliprack.rotateClipStart();
        cliprack.clipArmUp();

        outtake.resetHeightMotor();
        outtake.setHeightExtensionTarget(200);
        outtake.clawOpen();
        outtake.horizontalPara();
        outtake.verticalUp();
        outtake.resetRotationMotor();
        outtake.setRotationTarget(10);

        timer = new Timer();
    }

    @Override
    public void loop() {

        //ALL OF GAMEPAD1 CONTROLS

        if (gamepad1.dpad_up) {
            intake.verticalUp();
        } else if (gamepad1.dpad_down) {
            intake.verticalDown();
        }
        if (gamepad1.dpad_left) {
            intake.horizontalPerpendicular();
        } else if (gamepad1.dpad_right) {
            intake.horizontalPara();
        }
        if (gamepad1.x) {
            intake.clawOpen();
        } else if (gamepad1.b) {
            intake.clawClose();
        }
        if(gamepad1.y) {
            setState(1); //Transfer
        }



        //GAMEPAD 2 CONTROLS

        if(gamepad2.dpad_up){
            setState(15);
        } else if (gamepad2.dpad_down){
            cliprack.rackDown();
        }
        if (gamepad2.b) {
            outtake.clawClose();
        }
        if(gamepad2.y){
            outtake.prepareScore();
        }
        if(gamepad2.a){
            outtake.clawOpen();
        }
        if (gamepad2.x) {
            outtake.clawCloseHard();
        }
        if(gamepad2.right_bumper){
            setState(6); //Get first clip ready
        }
        if(gamepad2.left_bumper){
            setState(9); //Clipping
        }
        if(gamepad2.dpad_left){
            outtake.neutral();
        }


//        outtakeHeighthTarget += (gamepad2.left_stick_y * 25);
//        outtakeHeighthTarget = Math.max(Math.min(outtakeHeighthTarget, outtake.heightTargetMax), 0);
//        outtake.setHeightExtensionTarget((int) outtakeHeighthTarget);

//        rotationalTarget += (gamepad2.right_stick_y * 10);
//        rotationalTarget = Math.max(Math.min(rotationalTarget, 750), -750);
//        outtake.setRotationTargetActual((int) rotationalTarget);


        //Intake extension changing, multiplied by such a small number cause its a servo on a linkage
        intakeExtensionTarget += (gamepad1.left_trigger * .01) - (gamepad1.right_trigger * .01);
        intakeExtensionTarget = Math.max(Math.min(intakeExtensionTarget, .8), .2);
        intake.setExtensionTarget(intakeExtensionTarget);

        //Updaters for all subsystems
        intake.update();
        cliprack.update();
        outtake.update();

        autonomousUpdate();

        //All Driving
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x, true);
        follower.update();

        /* Telemetry Outputs of the Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /* Update Telemetry */
        intake.sendTelemetry(telemetry);
        cliprack.sendTelemetry(telemetry);
        outtake.sendTelemetry(telemetry);
        telemetry.update();


        }

        public void setUpTransferClaws(){
            intake.verticalUp();
            intake.horizontalPara();
            intake.clawClose();
            outtake.horizontalPara();
            outtake.clawOpen();
            outtake.verticalUp();
        }

        public void setUpTransferSlides(){
            outtake.setHeightExtensionTarget(30);
            intakeExtensionTarget = 0;
            outtake.setRotationTargetActual(10);
        }

    void setState(int num) {
        transferCase = num;
        timer.reset();
    }

    public void autonomousUpdate(){

        switch(transferCase){
            case 0:
                break;
            case 1:
                setUpTransferClaws();
                setState(2);
                break;
            case 2:
                if(timer.getTimeSeconds() > 0) {
                    intake.verticalTransfer();
                    intake.horizontalPerpendicular();
                    setUpTransferSlides();
                    setState(3);
                }
                break;
            case 3:
                if(timer.getTimeSeconds() > .5) {
                    outtake.setVerticalTransfer();
                    outtake.horizontalTransfer();
                    intake.clawCloseHard();
                    setState(4);
                }
                break;
            case 4:
                if(timer.getTimeSeconds() > 1) {
                    outtake.setRotationTargetActual(80);
                }  if (timer.getTimeSeconds() > 1.5){
                    outtake.clawClose();
                    setState(5);
                }
                break;
            case 5:
                if(timer.getTimeSeconds() > .5) {
                    intake.clawOpen();
                    outtake.horizontalPara();

                }
                 if(timer.getTimeSeconds() > .75){
                     outtake.clawCloseHard();
                    intake.setSetUp();
                    setState(0);
                }
                break;




            case 6:
                cliprack.readyClip();
                setState(7);
                break;
            case 7:
                if(timer.getTimeSeconds() > .25){
                    cliprack.raiseRight();
                    setState(8);
                }
                break;
            case 8:
                if(timer.getTimeSeconds() > 1){
                    cliprack.rackDown();
                    cliprack.rotateClipReady();
                    setState(0);
                }
                break;



            case 9:
                outtake.verticalCustom(.6);
                setState(10);
                break;
            case 10:
                if(timer.getTimeSeconds() > .5){
                    outtake.clawCloseHard();
                    outtake.horizontalCustom(.78);
                    cliprack.rotateClipReady();
                    setState(11);
                }
                break;
            case 11:
                if(timer.getTimeSeconds() > 1) {
                    outtake.setHeightExtensionTarget(235);
                    outtake.setRotationTargetActual(-150);
                    outtake.verticalCustom(.7);
                    setState(12);
                }
                break;
            case 12:
                if(timer.getTimeSeconds() > .75) {
                    cliprack.rotateClipStop();
                    setState(13);
                }
                break;
            case 13:
                if(timer.getTimeSeconds() > .75) {
                    cliprack.clipArmDown();
                    cliprack.rotateClipReady();
                    setState(14);
                }
                break;
            case 14:
                if(timer.getTimeSeconds() > 1) {
                    outtake.prepareScore();
                    setState(6);
                }
                break;



            case 15:
                cliprack.rackMiddle();
                setState(16);
                break;
            case 16:
                if(timer.getTimeSeconds() > .5) {
                    cliprack.rackUp();
                    setState(0);
                }
                break;
        }







    }



    }
