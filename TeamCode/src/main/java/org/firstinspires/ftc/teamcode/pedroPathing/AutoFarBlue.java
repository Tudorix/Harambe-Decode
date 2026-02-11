package org.firstinspires.ftc.teamcode.pedroPathing;



import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Threads.Motors;
import org.firstinspires.ftc.teamcode.Threads.Selectioner;
import org.firstinspires.ftc.teamcode.Threads.Servos;
import org.firstinspires.ftc.teamcode.Threads.Localization;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.HardwareClass;


@Autonomous(name = "Auto Far Blue" , group = "Test")
public class AutoFarBlue extends OpMode {
    int target = 0;
    private final Pose startPose = new Pose(133.6, 57.05, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(123.5, 57.3,Math.toRadians(240)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scorePoseError = new Pose(123.5,54,Math.toRadians(242.5)); // 116??
    private final Pose pickupScore1 = new Pose(108.42,43,Math.toRadians(270));
    private final Pose pickupScore1_3 = new Pose(108.42,14,Math.toRadians(270));
    private final Pose pickupPose2 = new Pose(131.09,11.74,Math.toRadians(288.39));
    private final Pose pickupBlind= new Pose(127,12.2,Math.toRadians(270));
    private final Pose parkPose = new Pose(131.4,34.5,Math.toRadians(270));
    private final ElapsedTime delay = new ElapsedTime();

    private Follower follower;

    Servos servos = null;
    private TelemetryManager telemetryM;
    Motors motors = null;
    Localization localization = null;
    Selectioner selectioner = null;
    HardwareClass hardwareClass = null;

    int smallDelay = 200; //mill
    int mediumDelay = 500;//mill

    int bigDelay = 1000;//mill

    ElapsedTime timer = new ElapsedTime();
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState, shoots = 0;
    private final int targetVelocity = 3000;
    int[] Pos = {0, 0, 0, 0, 0, 0};
    double targetAngle, targetPosition;

    private double power = 0.64;


    private Path scorePreload;
    private PathChain park, preload,pickup1, pickup1_3,scorePickup,pickupScore2,scoreScore2;


    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        preload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickupScore1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickupScore1.getHeading())
                .build();

        pickup1_3 = follower.pathBuilder()
                .addPath(new BezierLine(pickupScore1, pickupScore1_3))
                .setLinearHeadingInterpolation(pickupScore1.getHeading(), pickupScore1_3.getHeading())
                .build();

        scorePickup = follower.pathBuilder()
                .addPath(new BezierLine(pickupScore1_3, scorePoseError))
                .setLinearHeadingInterpolation(pickupScore1_3.getHeading(), scorePoseError.getHeading())
                .build();

        pickupScore2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseError, pickupPose2))
                .setLinearHeadingInterpolation(scorePoseError.getHeading(), pickupPose2.getHeading())
                .build();

        scoreScore2 = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose2, scorePoseError))
                .setLinearHeadingInterpolation(pickupPose2.getHeading(), scorePoseError.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseError, parkPose))
                .setLinearHeadingInterpolation(scorePoseError.getHeading(), parkPose.getHeading())
                .build();

    }


    /* You could check for
               - Follower State: "if(!follower.isBusy()) {}"
               - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
               - Robot Position: "if(follower.getPose().getX() > 36) {}"
               */
    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(preload);
                motors.setRampVelocityC(2950);
                servos.hoodMove(15);
                servos.turretGT(0.34);
                follower.setMaxPower(0.8);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds()>4) {
                        shootBall();
                        sleep(mediumDelay);
                        follower.followPath(pickup1, true);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds()>1.5) {
                        motors.intakeOn();
                        follower.setMaxPower(0.85);
                        servos.turretGT(0.34);// 0.66?
                        follower.followPath(pickup1_3, true);
                        setPathState(3);
                    }
                }
                break;

            case 3:

                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds()>2.7) {
                        motors.intakeReverse();
                        follower.followPath(scorePickup, true);
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    motors.intakeOff();
                    if(pathTimer.getElapsedTimeSeconds()>4.5) {
                        servos.hoodMove(16);
                        shootBall();
                        motors.intakeOn();
                        follower.followPath(pickupScore2, true);
                        setPathState(5);
                    }
                }
                break;

            case 5:{
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds()>4.5) {
                        motors.intakeReverse();
                        servos.turretGT(0.35);// 0.66?
                        follower.followPath(scoreScore2, true);
                        setPathState(6);
                    }
                }
                break;
            }
            case 6:{
                if(timer.milliseconds()>400)
                    motors.intakeOff();
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds()>3) {
                        shootBall();
                        follower.followPath(park, true);        //Note: mai in dreapta, ii prea jos
                        setPathState(100);
                    }
                }
                break;
            }
            case 100:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds()>1) {
                        PoseStorage.autoPose = follower.getPose();
                        selectioner.stop();
                        setPathState(-1);
                        motors.rampStop();
                    }
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


        telemetry.addData("path state", pathState);
        telemetry.addData("Ball1: ", Pos[1]);
        telemetry.addData("Ball2: ", Pos[2]);
        telemetry.addData("Ball3: ", Pos[3]);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Power: ",power);
        telemetryM.addData("RPM: ",motors.getVelocity());
        telemetry.addData("Shoots: ",shoots);
        telemetryM.addData("Shoots:", shoots);
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
        localization = Localization.getInstance(hardwareMap,telemetry);
        selectioner =Selectioner.getInstance(hardwareClass,telemetry);
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        hardwareClass.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareClass.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareClass.intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        selectioner.init();
        selectioner.start();
        selectioner.aladam();
        servos.hoodMove(1);
        follower.setStartingPose(startPose);
        motors.setRampCoefs();
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
    public void handleTurret() {
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double dx=0,dy=0;


        if(target == 1) {
            dx = HardwareClass.blueScoreX - x;
            dy = HardwareClass.blueScoreY - y;
        }

        if(target == 0) {
            dx = HardwareClass.redScoreX - x;
            dy = HardwareClass.redScoreY - y;
        }
        double goalAngle = Math.atan2(dy, dx);
        double thetaR = follower.getPose().getHeading();

        targetAngle = goalAngle - thetaR;
        targetAngle = Math.atan2(Math.sin(targetAngle), Math.cos(targetAngle));
        targetPosition = convertToNewRange(
                targetAngle,
                Math.PI / 2, -Math.PI / 2,
                0.19, 0.81
        );
//        if(localization.checkResults())
//            targetPosition+=(localization.getXPos()*0.01);
        targetPosition = Math.min(Math.max(targetPosition,0.19),0.81);
        servos.turretGT(targetPosition);

    }

    public double convertToNewRange(double value,
                                    double oldMin, double oldMax,
                                    double newMin, double newMax) {
        return newMin + (value - oldMin) * (newMax - newMin) / (oldMax - oldMin);
    }

    private void waitForRpm(){
        if(delay.seconds()>1.3){
            if (Math.abs(motors.getVelocity() * 60 / 28 - targetVelocity) < 100)
                shootBall();
            delay.reset();
        }
    }

    public void shootBall(){
        selectioner.unloadBallsSlow();
    }
    @Override
    public void stop() {
        selectioner.stop();
    }


}