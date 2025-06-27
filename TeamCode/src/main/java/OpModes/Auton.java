package OpModes;

import android.provider.SyncStateContract;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

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

    public PathChain readyClips, getClips, goodClips, goScore, preloadScore, clipsToSample1, firstSampleToScore, secondSample, thirdSample, secondSampleToScore, intakeHumanPlayer, scoreFromHumanPlayer;
    Follower follower;

    public static Pose startPose = new Pose(9, 56, Math.toRadians(0));
    Pose scorePose = new Pose(36, 72, Math.toRadians(0));

    Pose clipReadyPose = new Pose(12, 12, Math.toRadians(0));
    Pose clipControlPoint = new Pose(42, 38);

    //TODO: SET THESE CONTROL POINT TO BE VALID
    Pose firstSampleScoreCP = new Pose(0, 0);
    Pose secondSampleScoreCP = new Pose(0, 0);
    Pose humanPlayerSampleCP = new Pose(0, 0);
    Pose clipPose = new Pose(7, 12, Math.toRadians(0));

    Pose firstSample = new Pose(26, 22);
    Pose secondSampleP = new Pose(26, 10);

    Pose humanPlayer = new Pose(24, 12, Math.toRadians(90));



    @Override
    public void runOpMode() throws InterruptedException {


        is = new IntakeSubsystem(hardwareMap);
        os = new OuttakeSubsystem(hardwareMap);
        cs = new CliprackSubsystem(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        follower.update();

        is.setSetUp();
        os.prepareScore();
        cs.setSetUp();
        buildPaths();


        //INIT
        while (!isStarted() && !isStopRequested()) {
            is.update();
            os.update();
            cs.update();
            follower.update();
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
                .addPath(new BezierLine(new Point(scorePose), new Point(clipReadyPose)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        getClips = follower.pathBuilder()
                .addPath(new BezierLine(new Point(clipReadyPose), new Point(clipPose)))
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

        preloadScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();

        clipsToSample1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(clipReadyPose), new Point(firstSample)))
                .setConstantHeadingInterpolation(clipReadyPose.getHeading())
                .build();

        firstSampleToScore = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(firstSample), new Point(firstSampleScoreCP), new Point(scorePose)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();

        secondSample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(secondSampleP)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();

        thirdSample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(secondSampleP)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Math.toRadians(285))
                .build();

        secondSampleToScore = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(secondSampleP), new Point(secondSampleScoreCP), new Point(scorePose)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();

        intakeHumanPlayer = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(humanPlayer)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), humanPlayer.getHeading())
                .build();

        scoreFromHumanPlayer = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(humanPlayer), new Point(humanPlayerSampleCP), new Point(scorePose)))
                .setLinearHeadingInterpolation(humanPlayer.getHeading(), scorePose.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: //Run forward to score preload
                follower.followPath(preloadScore);
                setPathState(1);
                break;

//            case 1: //After robot has ran into bar with specimen, open claw and move to get clips
//                if(!follower.isBusy()){
//                    os.clawOpen();
//                    if(pathTimer.getTimeSeconds() > .25){
//                        os.neutral();
//                        follower.followPath(readyClips);
//                        setPathState(2);
//                    }
//                }
//                break;
//
//            case 2: //After robot has moved into the ready position, move back the rain rack
//                if(!follower.isBusy()){
//                    follower.followPath(getClips);
//                    setPathState(3);
//                }
//                break;
//
//            case 3: //After the robot moves back, raise rack and then move forward
//                if(!follower.isBusy()){
//                    setState(15); //Raise rack
//                    if(pathTimer.getTimeSeconds() > .5){
//                        follower.followPath(goodClips);
//                        setPathState(4);
//                    }
//                }
//                break;
//            case 4: //Robot has grabbed the clips and come forward, bring rack down then move to grab first sample
//                if(!follower.isBusy()) {
//                    cs.rackDown();
//                    is.prepIntake();
//                    is.setExtensionTarget(1);
//                    follower.followPath(clipsToSample1);
//                    setPathState(5);
//                }
//                break;
//
//            case 5: //After it gets to hovering over the sample, grab and then transfer.
//                if(!follower.isBusy()){
//                    is.clawClose();
//                    setState(6); //Get first clip on the thingy
//                    setPathState(6);
//                }
//                break;
//            case 6:
//                if(pathTimer.getTimeSeconds() > .125){
//                    setState(1); //Transfer
//                    setPathState(7);
//                }
//                break;
//            case 7: //After transfer, clip
//                if(pathTimer.getTimeSeconds() > 2){
//                    setState(9);
//                    setPathState(8);
//
//                }
//                break;
//            case 8: //After clipping is done, move to score first (2nd) spec
//                if(pathTimer.getTimeSeconds() > 2){
//                    os.prepareScore();
//                    follower.followPath(firstSampleToScore);
//                    setPathState(9);
//                }
//                break;
//
//            case 9: //After scored, open claw and run to second sample
//                if(!follower.isBusy()){
//                    os.clawOpen();
//                    if(pathTimer.getTimeSeconds() > 1.5){
//                        os.neutral();
//                        is.prepIntake();
//                        follower.followPath(secondSample);
//                        setPathState(10);
//                    }
//                }
//            case 10: //After getting there, intake
//                if(!follower.isBusy()) {
//                    is.clawClose();
//                    setPathState(11);
//                }
//                break;
//            case 11:
//                if(pathTimer.getTimeSeconds() > .125){
//                    setState(1); //Transfer
//                    setPathState(12);
//                }
//                break;
//            case 12: //After transfer, clip 2nd sample
//                if(pathTimer.getTimeSeconds() > 2){
//                    setState(9);
//                    setPathState(13);
//                }
//                break;
//            case 13: //Score
//                if(pathTimer.getTimeSeconds() > 2){
//                    os.prepareScore();
//                    follower.followPath(secondSampleToScore);
//                    setPathState(14);
//                }
//                break;
//            case 14: //AFter score, run away for third
//                if(!follower.isBusy()){
//                    os.clawOpen();
//                    if(pathTimer.getTimeSeconds() > 1.5){
//                        os.neutral();
//                        is.prepIntake();
//                        follower.followPath(thirdSample);
//                        setPathState(15);
//                    }
//                }
//                break;
//            case 15: //After getting there, intake
//                if(!follower.isBusy()) {
//                    is.clawClose();
//                    setPathState(16);
//                }
//                break;
//            case 16:
//                if(pathTimer.getTimeSeconds() > .125){
//                    setState(1); //Transfer
//                    setPathState(17);
//                }
//                break;
//            case 17: //After transfer, clip 3rd sample
//                if(pathTimer.getTimeSeconds() > 2){
//                    setState(9);
//                    setPathState(18);
//                }
//                break;
//            case 18: //Score
//                if(pathTimer.getTimeSeconds() > 2){
//                    os.prepareScore();
//                    follower.followPath(secondSampleToScore);
//                    setPathState(19);
//                }
//                break;
//            case 19: //Score 4th, go grab from human player
//                if(!follower.isBusy()){
//                    os.clawOpen();
//                    if(pathTimer.getTimeSeconds() > 1.5){
//                        os.neutral();
//                        is.prepIntake();
//                        follower.followPath(intakeHumanPlayer);
//                        setPathState(20);
//                    }
//                }
//                break;
//            case 20: //After getting there, intake
//                if(!follower.isBusy()) {
//                    is.clawClose();
//                    setPathState(21);
//                }
//                break;
//            case 21:
//                if(pathTimer.getTimeSeconds() > .125){
//                    setState(1); //Transfer
//                    setPathState(22);
//                }
//                break;
//            case 22: //After transfer, clip 3rd sample
//                if(pathTimer.getTimeSeconds() > 2){
//                    setState(9);
//                    setPathState(23);
//                }
//                break;
//            case 23:
//                if(pathTimer.getTimeSeconds() > 2){
//                    os.prepareScore();
//                    follower.followPath(scoreFromHumanPlayer);
//                    setPathState(24);
//                }
//                break;
//            case 24:
//                if(!follower.isBusy()){
//                    os.clawOpen();
//                    if(pathTimer.getTimeSeconds() > 1.5){
//                        os.neutral();
//                        is.setSetUp();
//                        follower.followPath(intakeHumanPlayer);
//                        setPathState(-1);
//                    }
//                }
//                break;
//
//
//
//            case -1:
//                break;


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
