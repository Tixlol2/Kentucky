package OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Util.CliprackSubsystem;
import Util.IntakeSubsystem;
import Util.OuttakeSubsystem;
import Util.Timer;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Auton", group = "Real")
public class Auton extends LinearOpMode {

    IntakeSubsystem is;
    OuttakeSubsystem os;
    CliprackSubsystem cs;

    int pathState = 0;
    Timer pathTimer = new Timer();

    int transferCase = 0;
    Timer timer = new Timer();

    public PathChain readyClips, getClips, goodClips, goScore;
    Follower follower;

    Pose startPose = new Pose(9, 56, Math.toRadians(270));
    Pose clipReadyPose = new Pose(12, 12, Math.toRadians(270));
    Point clipControlPoint = new Point(42, 38);
    Pose clipPose = new Pose(7, 12, Math.toRadians(270));
    Pose scorePose = new Pose(36, 72, Math.toRadians(270));

    @Override
    public void runOpMode() throws InterruptedException {


        is = new IntakeSubsystem(hardwareMap);
        os = new OuttakeSubsystem(hardwareMap);
        cs = new CliprackSubsystem(hardwareMap);

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        is.setSetUp();
        os.setSetUp();
        cs.setSetUp();
        buildPaths();


        //INIT
        while (!isStarted() && !isStopRequested()) {
            is.update();
            os.update();
            cs.update();
            follower.setStartingPose(startPose);
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Follower X ", follower.getPose().getX());
            telemetry.addData("Follower Y ", follower.getPose().getY());
            telemetry.addData("Follower Heading ", follower.getPose().getHeading());
            telemetry.update();
        }

        //RUNNING
        while (opModeIsActive() && !isStopRequested()) {
            follower.update();
            autonomousPathUpdate();
            is.update();
            os.update();
            cs.update();
            follower.telemetryDebug(telemetry);
            telemetry.addData("Path State ", pathState);
            telemetry.update();
        }


    }

    public void buildPaths() {

        readyClips = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose), clipControlPoint, new Point(clipReadyPose)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        getClips = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(clipReadyPose), new Point(clipPose)))
                .setConstantHeadingInterpolation(clipReadyPose.getHeading())
                .build();

        goodClips = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(clipPose), new Point(clipReadyPose)))
                .setConstantHeadingInterpolation(clipPose.getHeading())
                .build();

        goScore = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(clipPose), new Point(scorePose)))
                .setConstantHeadingInterpolation(clipPose.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move from start to scoring position
                follower.followPath(readyClips);
                setPathState(1);
                break;

            case 1: // Wait until the robot is near the scoring position
                if (!follower.isBusy()) {
                    follower.setMaxPower(.3);
                    follower.followPath(getClips, true);
                    setPathState(2);
                }
                break;

            case 2: // Wait until the robot is near the first sample pickup position
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    cs.rackUp();
                    if(pathTimer.getTimeSeconds() > 1){
                    follower.followPath(goodClips, true);
                    setPathState(3);
                    }
                }
                break;

            case 3: // Wait until the robot returns to the scoring position
                if (pathTimer.getTimeSeconds() > 1) {
                    cs.rackDown();
                    follower.followPath(goScore, true);
                    setPathState(4);
                }
                break;

        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }

    public void autonomousUpdate() {

        switch (transferCase) {
            case 0:
                break;
            case 1:
                setUpTransferClaws();
                setState(2);
                break;
            case 2:
                if (timer.getTimeSeconds() > 0) {
                    is.verticalTransfer();
                    is.horizontalPerpendicular();
                    setUpTransferSlides();
                    setState(3);
                }
                break;
            case 3:
                if (timer.getTimeSeconds() > .5) {
                    os.setVerticalTransfer();
                    os.horizontalTransfer();
                    is.clawCloseHard();
                    setState(4);
                }
                break;
            case 4:
                if (timer.getTimeSeconds() > 1) {
                    os.setRotationTargetActual(80);
                }
                if (timer.getTimeSeconds() > 1.5) {
                    os.clawClose();
                    setState(5);
                }
                break;
            case 5:
                if (timer.getTimeSeconds() > .5) {
                    is.clawOpen();
                    os.horizontalPara();

                }
                if (timer.getTimeSeconds() > .75) {
                    os.clawCloseHard();
                    is.setSetUp();
                    setState(0);
                }
                break;


            case 6:
                cs.readyClip();
                setState(7);
                break;
            case 7:
                if (timer.getTimeSeconds() > .25) {
                    cs.raiseRight();
                    setState(8);
                }
                break;
            case 8:
                if (timer.getTimeSeconds() > 1) {
                    cs.rackDown();
                    cs.rotateClipReady();
                    setState(0);
                    break;
                }


            case 9:
                os.verticalCustom(.6);
                setState(10);
                break;
            case 10:
                if (timer.getTimeSeconds() > .5) {
                    os.clawCloseHard();
                    os.horizontalCustom(.78);
                    cs.rotateClipReady();
                    setState(11);
                }
                break;
            case 11:
                if (timer.getTimeSeconds() > 1) {
                    os.setHeightExtensionTarget(235);
                    os.setRotationTargetActual(-150);
                    os.verticalCustom(.7);
                    setState(12);
                }
                break;
            case 12:
                if (timer.getTimeSeconds() > .75) {
                    cs.rotateClipStop();
                    setState(13);
                }
                break;
            case 13:
                if (timer.getTimeSeconds() > .5) {
                    cs.clipArmDown();
                    cs.rotateClipReady();
                    setState(14);
                }
                break;
            case 14:
                if (timer.getTimeSeconds() > 1) {
                    os.prepareScore();
                    setState(6);
                }
                break;


            case 15:
                cs.rackMiddle();
                setState(16);
                break;
            case 16:
                if (timer.getTimeSeconds() > .5) {
                    cs.rackUp();
                    setState(0);
                }
                break;
        }


    }

    void setState ( int num){
        transferCase = num;
        timer.reset();
    }
    public void setUpTransferClaws () {
        is.verticalUp();
        is.horizontalPara();
        is.clawClose();
        os.horizontalPara();
        os.clawOpen();
        os.verticalUp();
    }

    public void setUpTransferSlides () {
        os.setHeightExtensionTarget(30);
        is.setExtensionTarget(0);
        os.setRotationTargetActual(10);
    }
}
