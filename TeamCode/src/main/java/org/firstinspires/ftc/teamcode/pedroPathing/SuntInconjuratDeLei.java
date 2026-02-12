package org.firstinspires.ftc.teamcode.pedroPathing;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareClass;
import org.firstinspires.ftc.teamcode.Threads.Limelight;
import org.firstinspires.ftc.teamcode.Threads.Motors;
import org.firstinspires.ftc.teamcode.Threads.Selectioner;
import org.firstinspires.ftc.teamcode.Threads.Servos;


@Autonomous(name = "Sunt Inconjurat De Lei" , group = "Test")
public class SuntInconjuratDeLei extends OpMode {
    int target=0;
    private final Pose startPose = new Pose(14.8, 118.7, 2.26); // Start Pose of our robot.
    private final Pose scorePose = new Pose(46.1, 97.7  , Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scorePose2 = new Pose(56.1, 97.7  , Math.toRadians(90)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scorePose3 = new Pose(60, 97.7  , Math.toRadians(90)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scorePose1 = new Pose(56.1,87.7,Math.toRadians(90)); // scorePose 1 doar cu turreta
    private final Pose pickup1Pose = new Pose(58, 99, Math.toRadians(90)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1_3Pose = new Pose(58, 120, Math.toRadians(90)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(82, 99   , Math.toRadians(90)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup2_3Pose = new Pose(78,128,Math.toRadians(90));
    private final Pose parkPose = new Pose(40.7, 91, 2.116); // Park // Park
    private final Pose unloadPose = new Pose(83,130,Math.toRadians(123.5));
    private final Pose curveAsist = new Pose(79,102,Math.toRadians(90));
    private final Pose rest = new Pose (68,87.5, Math.toRadians(90));
    private final Pose aux = new Pose (78, 90   , Math.toRadians(90));
    double shoots = 0;
    private Follower follower;
    private final ElapsedTime delay = new ElapsedTime();
    Servos servos = null;
    Motors motors = null;
    Limelight limelight = null;
    Selectioner selectioner = null;
    HardwareClass hardwareClass= null;
    private TelemetryManager telemetryM;

    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime path = new ElapsedTime();
    double targetAngle, targetPosition,distance;
    private Path scorePreload;
    private PathChain grabPickup1, grabPickup1_3, grabPickup2, grabPickup2_3,scorePickup2,scorePickup3, preload,scorePickup1, Park,rPath1;

    private PathChain unload1,unload2;
    private PathChain ScorePreload , GrabFirst, GrabFromRack, ScoreFromRack, GrabSecond;

    double Treshold = 0.73;
    int REP = 3000;
    double TargetRPM = 1850;

    int High_P = 60 , Low_P = 20;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        ScorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0,() -> {
                    startPresiune();
                    selectioner.resetServos();
                    servos.turretGT(0.52);
                })
                .build();


        GrabFirst = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .addParametricCallback(0,() -> {
                    motors.intakeOn();
                    selectioner.resetServos();
                })
                .addPath(new BezierLine(pickup2Pose, pickup2_3Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2_3Pose.getHeading())
                .addPath(new BezierLine(pickup2_3Pose, pickup2Pose))
                .setLinearHeadingInterpolation(pickup2_3Pose.getHeading(), pickup2Pose.getHeading())
                .addParametricCallback(0.4,() -> {
                    motors.intakeReverse();
                    servos.turretGT(0.66);
                })
                .addPath(new BezierLine(pickup2Pose, scorePose2))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose2.getHeading())
                .addParametricCallback(0.9,() -> {
                    startPresiune();
                    motors.intakeOff();
                })
                .build();

        GrabSecond = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickup1Pose.getHeading())
                .addParametricCallback(0,() -> {
                    motors.intakeOn();
                    selectioner.resetServos();
                })
                .addPath(new BezierLine(pickup1Pose, pickup1_3Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1_3Pose.getHeading())
                .addPath(new BezierLine(pickup1_3Pose, pickup1Pose))
                .setLinearHeadingInterpolation(pickup1_3Pose.getHeading(), pickup1Pose.getHeading())
                .addParametricCallback(0.3,() -> {
                    motors.intakeReverse();
                    servos.turretGT(0.67);
                })
                .addPath(new BezierLine(pickup1Pose, scorePose2))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose2.getHeading())
                .addParametricCallback(0.9,() -> {
                    motors.intakeOff();
                })
                .build();

        GrabFromRack = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickup2Pose.getHeading())
                .addPath(new BezierLine(pickup2Pose, aux))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), aux.getHeading())
                .addParametricCallback(0,() -> {
                    motors.intakeOn();
                    selectioner.resetServos();
                })

                .build();

        ScoreFromRack = follower.pathBuilder()
                .addPath(new BezierLine(unloadPose, pickup2Pose))
                .setLinearHeadingInterpolation(unloadPose.getHeading(), pickup2Pose.getHeading())
                .addParametricCallback(0.1,() -> {
                    motors.intakeReverse();
                })
                .addPath(new BezierLine(pickup2Pose, scorePose2))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose2.getHeading())
                .addParametricCallback(0.9,() -> {
                    startPresiune();
                    motors.intakeOff();
                })
                .build();

        grabPickup2_3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, pickup2_3Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup2_3Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2_3Pose,curveAsist, scorePose1))
                .setLinearHeadingInterpolation(pickup2_3Pose.getHeading(), scorePose1.getHeading())
                .build();

        unload1 = follower.pathBuilder()
                .addPath(new BezierLine(aux, unloadPose))
                .setLinearHeadingInterpolation(aux.getHeading(), unloadPose.getHeading())
                .build();

        unload2 = follower.pathBuilder()
                .addPath(new BezierLine(curveAsist,unloadPose))
                .setLinearHeadingInterpolation(curveAsist.getHeading(), unloadPose.getHeading())
                .build();

        rPath1 = follower.pathBuilder()
                .addPath(new BezierCurve(unloadPose,curveAsist,rest))
                .setLinearHeadingInterpolation(unloadPose.getHeading(), rest.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(rest, pickup1Pose))
                .setLinearHeadingInterpolation(rest.getHeading(), pickup1Pose.getHeading())
                .build();

        grabPickup1_3  = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, pickup1_3Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1_3Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1_3Pose, parkPose))
                .setLinearHeadingInterpolation(pickup1_3Pose.getHeading(), parkPose.getHeading())
                .build();

        Park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose1, parkPose))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), parkPose.getHeading())
                .build();

    }



    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(ScorePreload);
                setPathState(1);
                break;

            case 1:

                if(!follower.isBusy()) {
                    motors.intakeOn();
                    for(int i = 0; i < REP ; i++){
                        if(motors.getVelocity() >= TargetRPM * Treshold){
                            break;
                        }
                    }
                    stopPresiune();
                    setPathState(2);
                }
                break;

            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(GrabFirst,true);
                    setPathState(3);
                }
                break;

            case 3:
                if(!follower.isBusy()) {
                    startPresiune();
                    motors.intakeOn();
                    for(int i = 0; i < REP ; i++){
                        if(motors.getVelocity() >= TargetRPM * Treshold){
                            break;
                        }
                    }
                    stopPresiune();
                    setPathState(4);
                }
                break;

            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(GrabFromRack, true);
                    setPathState(5);
                }

                break;

            case 5:

                if(!follower.isBusy()) {
                    follower.followPath(unload1);
                    this.sleep(2000);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(ScoreFromRack,true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    motors.intakeOn();
                    for(int i = 0; i < REP ; i++){
                        if(motors.getVelocity() >= TargetRPM * Treshold){
                            break;
                        }
                    }
                    stopPresiune();
                    setPathState(8);
                }
                break;
            case 8:

                if(!follower.isBusy()) {
                    follower.followPath(GrabSecond,true);
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    startPresiune();
                    motors.intakeOn();
                    for(int i = 0; i < REP ; i++){
                        if(motors.getVelocity() >= TargetRPM * Treshold){
                            break;
                        }
                    }
                    stopPresiune();
                    setPathState(-1);
                }
                break;

            case 10:

                if(!follower.isBusy()) {

                    if(pathTimer.getElapsedTimeSeconds()>1.4) {
                        motors.intakeReverse();
                        motors.intakeOff();
                        shootBall();
                        sleep(150);
                        setPathState(100);
                    }
                }
                break;

            case 100:

                if(!follower.isBusy()) {
                    motors.rampStop();

                    PoseStorage.autoPose = follower.getPose();
                    setPathState(-1);
                }
                break;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        distance = limelight.getDistanceOD(
                follower.getPose().getX(),
                follower.getPose().getY(),
                target
        );
        //handleTurret();
        //servos.hoodMove((int)getHood(distance));
        //motors.setRampVelocityC((int)getRPM(distance));
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("RPM: ",motors.getVelocity());
        telemetry.addData("Distance to target:",distance);
        telemetry.addData("Timer:", timer.milliseconds());
        telemetry.update();
    }


    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        hardwareClass = HardwareClass.getInstance(hardwareMap);
        servos = Servos.getInstance(hardwareMap,telemetry);
        motors = Motors.getInstance(hardwareMap);
        limelight = Limelight.getInstance(hardwareMap,telemetry);
        limelight.setup();
        limelight.setPipeline(1);
        selectioner = Selectioner.getInstance(hardwareClass,telemetry);
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        hardwareClass.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareClass.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareClass.intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        //servos.hoodMove(1);
        follower.setStartingPose(startPose);
        motors.setRampCoefs();
        distance = limelight.getDistanceOD(follower.getPose().getX(), follower.getPose().getY(),target);
    }


    @Override
    public void init_loop() {}


    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    private void sleep(int delay){
        try {
            Thread.sleep(delay);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    private void waitForRpm(){

        telemetry.clear();
        telemetry.addData("Velocity:",motors.getVelocity()*60/28);
        telemetry.update();

        if(delay.seconds()>1){
            if (Math.abs(motors.getRampError(2500)) < 75)
                shootBall();
            delay.reset();
        }
    }
    public void shootBall(){
        selectioner.unloadBallsSlow();
        sleep(300);
        shoots++;
    }

    public void startPresiune(){
        motors.setCoefsMan(High_P,0,0,1.2);
        motors.setRampVelocityC((int)(TargetRPM));
        //selectioner.resetServos();
    }

    public void stopPresiune(){
        selectioner.unloadBallsQuick();
        motors.setCoefsMan(Low_P,0,0,1.2);
        motors.setRampVelocityC((int)(0.33 * TargetRPM));
        selectioner.resetServos();
        motors.intakeOff();
    }



    @Override
    public void stop() {
        //selectioner.stop();
    }


}