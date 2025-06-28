package OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
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

@Config
@Autonomous(name = "Auton", group = "Real")
public class Auton extends LinearOpMode {

    IntakeSubsystem intakeSubsystem;
    OuttakeSubsystem outtakeSubsystem;
    CliprackSubsystem cliprackSubsystem;

    int pathState = 0;
    Timer pathTimer = new Timer();

    int transferCase = 0;
    Timer timer = new Timer();

    public PathChain readyClips, getClips, goodClips, goScore, preloadScore, clipsToSample1, firstSampleToScore, secondSample, thirdSample, secondSampleToScore, intakeHumanPlayer, scoreFromHumanPlayer;
    Follower follower;

    public static Pose startPose = new Pose(9, 56, Math.toRadians(0));
    Pose scorePose = new Pose(38, 72, Math.toRadians(0));

    Pose clipReadyPose = new Pose(12, 12, Math.toRadians(0));
    Pose clipControlPoint = new Pose(42, 38);

    //TODO: SET THESE CONTROL POINT TO BE VALID
    Pose firstSampleScoreCP = new Pose(4.6, 77);
    Pose secondSampleScoreCP = new Pose(4.6, 77);
    Pose humanPlayerSampleCP = new Pose(4.6, 77);
    Pose clipPose = new Pose(10, 12, Math.toRadians(0));

    Pose firstSample = new Pose(30, 22);
    Pose secondSampleP = new Pose(30, 10);

    Pose humanPlayer = new Pose(24, 12, Math.toRadians(180));



    @Override
    public void runOpMode() throws InterruptedException {


        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        outtakeSubsystem = new OuttakeSubsystem(hardwareMap);
        cliprackSubsystem = new CliprackSubsystem(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        follower.update();

        intakeSubsystem.setSetUp();
        outtakeSubsystem.setSetUp();
        cliprackSubsystem.setSetUp();
        buildPaths();


        //INIT
        while (!isStarted() && !isStopRequested()) {
            intakeSubsystem.update();
            outtakeSubsystem.update();
            cliprackSubsystem.update();
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
            autonomousUpdate();
            intakeSubsystem.update();
            outtakeSubsystem.update();
            cliprackSubsystem.update();
            telemetry.addData("Path State ", pathState);
            telemetry.addData("Follower X ", follower.getPose().getX());
            telemetry.addData("Follower Y ", follower.getPose().getY());
            telemetry.addData("Follower Heading ", follower.getPose().getHeading());
            telemetry.addData("Follower isBusy ", follower.isBusy());
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
                outtakeSubsystem.prepareScore();
                follower.followPath(preloadScore, true);
                setPathState(1);
                break;

            case 1: //After robot has ran into bar with specimen, open claw and move to get clips
                if(!follower.isBusy()){
                    outtakeSubsystem.clawOpen();
                    if(pathTimer.getTimeSeconds() > .25){
                        outtakeSubsystem.neutral();
                        follower.followPath(readyClips, true);
                        setPathState(2);
                    }
                }
                break;

            case 2: //After robot has moved into the ready position, move back the rain rack
                if(!follower.isBusy()){
                    follower.followPath(getClips, true);
                    setPathState(3);
                }
                break;

            case 3: //After the robot moves back, raise rack and then move forward
                if(!follower.isBusy()){
                    setState(15); //Raise rack
                    if(pathTimer.getTimeSeconds() > 2.5){
                        follower.followPath(goodClips,true);
                        setPathState(4);
                    }
                }
                break;
            case 4: //Robot has grabbed the clips and come forward, bring rack down then move to grab first sample
                if(!follower.isBusy()) {
                    cliprackSubsystem.rackDown();
                    intakeSubsystem.prepIntake();
                    intakeSubsystem.setExtensionTarget(1);
                    follower.followPath(clipsToSample1,true);
                    setPathState(5);
                }
                break;

            case 5: //After it gets to hovering over the sample, grab and then transfer.
                if(!follower.isBusy()){
                    intakeSubsystem.clawClose();
                    setState(6); //Get first clip on the thingy
                    setPathState(6);
                }
                break;
            case 6:
                if(pathTimer.getTimeSeconds() > .25){
                    setState(1); //Transfer
                    setPathState(7);
                }
                break;
            case 7: //After transfer, clip
                if(pathTimer.getTimeSeconds() > 2){
                    setState(9);
                    setPathState(8);

                }
                break;
            case 8: //After clipping is done, move to score first (2nd) spec
                if(pathTimer.getTimeSeconds() > 2){
                    outtakeSubsystem.prepareScore();
                    follower.followPath(firstSampleToScore,true);
                    setPathState(9);
                }
                break;

            case 9: //After scored, open claw and run to second sample
                if(!follower.isBusy() && follower.atPose(scorePose, .5, .5)){
                    outtakeSubsystem.clawOpen();
                    if(pathTimer.getTimeSeconds() > 1.5){
                        outtakeSubsystem.neutral();
                        intakeSubsystem.prepIntake();
                        follower.followPath(secondSample,true);
                        setPathState(10);
                    }
                }
            case 10: //After getting there, intake
                if(!follower.isBusy() && follower.atPose(secondSampleP, .5, .5)) {
                    intakeSubsystem.clawClose();
                    setPathState(11);
                }
                break;
            case 11:
                if(pathTimer.getTimeSeconds() > .125){
                    setState(1); //Transfer
                    setPathState(12);
                }
                break;
            case 12: //After transfer, clip 2nd sample
                if(pathTimer.getTimeSeconds() > 2){
                    setState(9);
                    setPathState(13);
                }
                break;
            case 13: //Score
                if(pathTimer.getTimeSeconds() > 2){
                    outtakeSubsystem.prepareScore();
                    follower.followPath(secondSampleToScore,true);
                    setPathState(14);
                }
                break;
            case 14: //AFter score, run away for third
                if(!follower.isBusy() && follower.atPose(scorePose, .5, .5)){
                    outtakeSubsystem.clawOpen();
                    if(pathTimer.getTimeSeconds() > 1.5){
                        outtakeSubsystem.neutral();
                        intakeSubsystem.prepIntake();
                        follower.followPath(thirdSample,true);
                        setPathState(15);
                    }
                }
                break;
            case 15: //After getting there, intake
                if(!follower.isBusy() && follower.atPose(secondSampleP, .5, .5)) {
                    intakeSubsystem.clawClose();
                    setPathState(16);
                }
                break;
            case 16:
                if(pathTimer.getTimeSeconds() > .125){
                    setState(1); //Transfer
                    setPathState(17);
                }
                break;
            case 17: //After transfer, clip 3rd sample
                if(pathTimer.getTimeSeconds() > 2){
                    setState(9);
                    setPathState(18);
                }
                break;
            case 18: //Score
                if(pathTimer.getTimeSeconds() > 2){
                    outtakeSubsystem.prepareScore();
                    follower.followPath(secondSampleToScore,true);
                    setPathState(19);
                }
                break;
            case 19: //Score 4th, go grab from human player
                if(!follower.isBusy() && follower.atPose(scorePose, .5, .5)){
                    outtakeSubsystem.clawOpen();
                    if(pathTimer.getTimeSeconds() > 1.5){
                        outtakeSubsystem.neutral();
                        intakeSubsystem.prepIntake();
                        follower.followPath(intakeHumanPlayer,true);
                        setPathState(20);
                    }
                }
                break;
            case 20: //After getting there, intake
                if(!follower.isBusy() && follower.atPose(humanPlayer, .5, .5)) {
                    intakeSubsystem.clawClose();
                    setPathState(21);
                }
                break;
            case 21:
                if(pathTimer.getTimeSeconds() > .125){
                    setState(1); //Transfer
                    setPathState(22);
                }
                break;
            case 22: //After transfer, clip 3rd sample
                if(pathTimer.getTimeSeconds() > 2){
                    setState(9);
                    setPathState(23);
                }
                break;
            case 23:
                if(pathTimer.getTimeSeconds() > 2){
                    outtakeSubsystem.prepareScore();
                    follower.followPath(scoreFromHumanPlayer,true);
                    setPathState(24);
                }
                break;
            case 24:
                if(!follower.isBusy() && follower.atPose(scorePose, .5, .5)){
                    outtakeSubsystem.clawOpen();
                    if(pathTimer.getTimeSeconds() > 1.5){
                        outtakeSubsystem.neutral();
                        intakeSubsystem.setSetUp();
                        follower.followPath(intakeHumanPlayer,true);
                        setPathState(-1);
                    }
                }
                break;



            case -1:
                break;


        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
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
                    intakeSubsystem.verticalTransfer();
                    intakeSubsystem.setCustomHorizontal(.5);
                    setUpTransferSlides();
                    setState(3);
                }
                break;
            case 3:
                if(timer.getTimeSeconds() > .5) {
                    outtakeSubsystem.setVerticalTransfer();
                    outtakeSubsystem.horizontalTransfer();
                    intakeSubsystem.clawCloseHard();
                    setState(4);
                }
                break;
            case 4:
                if(timer.getTimeSeconds() > 1) {
                    outtakeSubsystem.setRotationTargetActual(80);
                }  if (timer.getTimeSeconds() > 1.5){
                outtakeSubsystem.clawClose();
                setState(5);
            }
                break;
            case 5:
                if(timer.getTimeSeconds() > .75) {
                    intakeSubsystem.clawOpen();
                    outtakeSubsystem.horizontalPara();

                }
                if(timer.getTimeSeconds() > 1){
                    outtakeSubsystem.clawCloseHard();
                    intakeSubsystem.setSetUp();
                    setState(0);
                }
                break;




            case 6:
                cliprackSubsystem.readyClip();
                setState(7);
                break;
            case 7:
                if(timer.getTimeSeconds() > .25){
                    cliprackSubsystem.raiseRight();
                    setState(8);
                }
                break;
            case 8:
                if(timer.getTimeSeconds() > 1){
                    cliprackSubsystem.rackDown();
                    cliprackSubsystem.rotateClipReady();
                    setState(0);
                }
                break;



            case 9:
                outtakeSubsystem.verticalCustom(.6);
                setState(10);
                break;
            case 10:
                if(timer.getTimeSeconds() > 1){
                    outtakeSubsystem.clawCloseHard();
                    outtakeSubsystem.horizontalCustom(.78);
                    cliprackSubsystem.rotateClipReady();
                    outtakeSubsystem.setHeightExtensionTarget(Teleop.heightClipping);
                    setState(11);
                }
                break;
            case 11:
                if(timer.getTimeSeconds() > 2) {
                    outtakeSubsystem.setRotationTargetActual(Teleop.rotationClipping);
                    outtakeSubsystem.verticalCustom(.7);
                    setState(12);
                }
                break;
            case 12:
                if(timer.getTimeSeconds() > 2.5) {
                    cliprackSubsystem.rotateClipStop();
                    setState(13);
                }
                break;
            case 13:
                if(timer.getTimeSeconds() > 2) {
                    cliprackSubsystem.clipArmDown();
                    cliprackSubsystem.rotateClipReady();
                    setState(14);
                }
                break;
            case 14:
                if(timer.getTimeSeconds() > 2) {
                    outtakeSubsystem.prepareScore();
                    setState(6);
                }
                break;



            case 15:
                cliprackSubsystem.rackMiddle();
                setState(16);
                break;
            case 16:
                if(timer.getTimeSeconds() > .125) {
                    cliprackSubsystem.rackUp();
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
        intakeSubsystem.verticalUp();
        intakeSubsystem.horizontalPara();
        intakeSubsystem.clawClose();
        outtakeSubsystem.horizontalPara();
        outtakeSubsystem.clawOpen();
        outtakeSubsystem.verticalUp();
    }

    public void setUpTransferSlides () {
        outtakeSubsystem.setHeightExtensionTarget(30);
        intakeSubsystem.setExtensionTarget(0);
        outtakeSubsystem.setRotationTargetActual(10);
    }
}
