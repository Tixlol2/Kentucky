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

    CliprackSubsystem cliprack;

    OuttakeSubsystem outtake;

    private int transferCase = 0;

    Timer timer;

    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
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
        if(gamepad1.right_stick_button) {
            setState(1);
        }
        if(gamepad1.left_stick_button){
            setState(6);
        }




        if(gamepad2.dpad_up){
            cliprack.rackUp();
        } else if (gamepad2.dpad_down){
            cliprack.rackDown();
        }
        if (gamepad2.circle){
            cliprack.rotateClipStart();
        } else if (gamepad2.square) {
            cliprack.rotateClipStop();
        }
        if (gamepad2.triangle){
            cliprack.clipArmUp();
        } else if (gamepad2.x) {
            cliprack.clipArmDown();
        }


        if(gamepad2.dpad_left){
            outtake.setHeightExtensionTarget(500);
            outtake.verticalDown();
            cliprack.rotateClipStart();
        }
        if(gamepad2.dpad_right){
            cliprack.raiseRight();
        }
        if(gamepad2.left_stick_button){
            outtake.clawCloseHard();
            cliprack.rotateClipReady();
            outtake.setRotationTarget(-30);
            cliprack.rackDown();
        }
        if(gamepad2.right_stick_button){
            cliprack.rotateClipStop();
        }



        //Intake extension changing, multiplied by such a small number cause its a servo on a linkage
        intakeExtensionTarget += (gamepad1.left_trigger * .005) - (gamepad1.right_trigger * .005);
        intakeExtensionTarget = Math.max(Math.min(intakeExtensionTarget, .8), .2    );
        intake.setExtensionTarget(intakeExtensionTarget);

        //Updaters for all subsystems
        intake.update();
        cliprack.update();
        outtake.update();

        autonomousUpdate();

        //All Driving
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
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
            outtake.setRotationTarget(5);
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
                    if(timer.getTimeSeconds() > 1) {
                        setUpTransferSlides();
                        setState(3);
                    }
                    break;
                case 3:
                    if(timer.getTimeSeconds() > 1) {
                        intake.verticalTransfer();
                        setState(4);
                    }
                    break;
                case 4:
                    if(timer.getTimeSeconds() > 1) {
                        outtake.horizontalTransfer();
                    }  if (timer.getTimeSeconds() > 2){
                        outtake.clawClose();
                        setState(5);
                    }
                    break;
                case 5:
                    if(timer.getTimeSeconds() > 1) {
                        intake.clawOpen();
                        outtake.horizontalPara();

                    }
                     if(timer.getTimeSeconds() > 3){
                        setState(0);
                    }
                    break;




                case 6:

                    setState(7);
                    break;
                case 7:

                    setState(8);
                    break;
                case 8:
                    if(timer.getTimeSeconds() > .5){

                        setState(9);
                        break;
                    }

            }







        }



    }
