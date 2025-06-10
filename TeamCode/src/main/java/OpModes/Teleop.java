package OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

        outtake.setHeightExtensionTarget(200);
        outtake.clawOpen();
        outtake.horizontalPara();
        outtake.verticalUp();
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
        } else {
            intake.clawClose();
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
        } else if (gamepad2.square) {
            cliprack.clipArmDown();
        }

        //Intake extension changing, multiplied by such a small number cause its a servo on a linkage
        intakeExtensionTarget += (gamepad1.left_trigger * .005) - (gamepad1.right_trigger * .005);
        intakeExtensionTarget = Math.max(Math.min(intakeExtensionTarget, .8), 0);
        intake.setExtensionTarget(intakeExtensionTarget);

        //Updaters for all subsystems
        intake.update();
        cliprack.update();
        outtake.update();

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
    }
