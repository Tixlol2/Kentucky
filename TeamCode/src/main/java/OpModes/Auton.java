package OpModes;

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

        is.setSetUp();
        os.setSetUp();
        cs.setSetUp();
        buildPaths();
        telemetry.addData("Status", "Initialized");
        telemetry.update();



        waitForStart();


        while(opModeIsActive() && !isStopRequested()) {
            autonomousPathUpdate();
            is.update();
            os.update();
            cs.update();
            telemetry.update();
        }



    }

    public void buildPaths(){

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
                    follower.followPath(goodClips, true);
                    setPathState(3);
                }
                break;

            case 3: // Wait until the robot returns to the scoring position
                if (!follower.isBusy()) {
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
}
