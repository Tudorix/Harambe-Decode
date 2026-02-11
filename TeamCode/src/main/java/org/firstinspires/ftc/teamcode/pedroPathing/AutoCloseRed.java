package org.firstinspires.ftc.teamcode.pedroPathing;



import com.bylazar.telemetry.PanelsTelemetry;
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

import org.firstinspires.ftc.teamcode.HardwareClass;
import org.firstinspires.ftc.teamcode.Threads.Motors;
import org.firstinspires.ftc.teamcode.Threads.Servos;
import org.firstinspires.ftc.teamcode.Threads.Selectioner;
import org.firstinspires.ftc.teamcode.Threads.Limelight;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Auto Close Red" , group = "Test")
public class AutoCloseRed extends OpMode {
    int target=0;
    private final Pose startPose = new Pose(14.8, 118.7, 2.26); // Start Pose of our robot.
    private final Pose scorePose = new Pose(56.1, 87.7  , Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scorePose1 = new Pose(56.1,87.7,Math.toRadians(90)); // scorePose 1 doar cu turreta
    private final Pose pickup1Pose = new Pose(61, 99, Math.toRadians(90)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1_3Pose = new Pose(61, 125, Math.toRadians(90)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(82, 99   , Math.toRadians(90)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup2_3Pose = new Pose(78,130,Math.toRadians(90));
    private final Pose parkPose = new Pose(40.7, 91, 2.116); // Park // Park
    private final Pose unloadPose = new Pose(82.4,132,Math.toRadians(123.5));
    private final Pose curveAsist = new Pose(79,102,Math.toRadians(90));
    private final Pose rest = new Pose (68,87.5, Math.toRadians(90));
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



    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        preload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
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
                .addPath(new BezierLine(scorePose1,curveAsist))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), curveAsist.getHeading())
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

                follower.followPath(preload);
                motors.setRampVelocityC(2150);
                servos.turretGT(0.52);
                servos.hoodMove(6);
                follower.setMaxPower(0.95);
                setPathState(1);
                break;

            case 1:

                if(!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.9) {
                        shootBall();
                        if (pathTimer.getElapsedTimeSeconds() > 3.2) {

                            follower.followPath(grabPickup2, true);
                            timer.reset();
                            setPathState(2);
                        }
                    }
                }
                break;

            case 2:

                if(!follower.isBusy()) {


                    motors.intakeOn();
                    if(timer.seconds()>0.6) {
                        follower.followPath(grabPickup2_3, true);
                        setPathState(3);
                        timer.reset();
                    }
                }
                break;

            case 3:

                if(!follower.isBusy()) {
                    motors.intakeOn();
                    if(timer.seconds()>1.2) {
                        servos.turretGT(0.63);
                        timer.reset();
                        servos.hoodMove(10);
                        follower.followPath(scorePickup2, true);
                        setPathState(4);
                    }
                }
                break;

            case 4:
                if(timer.seconds()>0.4)
                    motors.intakeReverse();
                if(timer.seconds()>0.63)
                    motors.intakeOff();
                if(!follower.isBusy()) {
                    motors.intakeOff();
                    shootBall();
                    motors.intakeOn();
                    servos.hoodMove(6);
                    follower.followPath(unload1, true);
                    setPathState(5);
                }
                break;

            case 5:

                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds()>1) {
                        motors.intakeOn();
                        timer.reset();
                        setPathState(6);
                        follower.followPath(unload2, true);
                    }
                    timer.reset();
                }
                break;
            case 6:
                motors.intakeOn();
                if(!follower.isBusy()) {
                    if(timer.seconds()>2.1) {
                        servos.turretGT(0.72);
//                        turret.goToPosition(-170);
//                        turret.goToPosition(-230);//255
                        timer.reset();
                        follower.followPath(rPath1,true);
                        setPathState(7);
                    }
                }
                break;
            case 7:
                if(timer.seconds()>0.3)
                    motors.intakeReverse();
                if(timer.seconds()>0.55)
                    motors.intakeOff();
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds()>2.3) {
                        shootBall();
                        sleep(200);
                        motors.intakeOff();
                        follower.followPath(grabPickup1, true);
                        setPathState(8);
                    }
                }
                break;
            case 8:

                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds()>0.7) {
                        timer.reset();
                        motors.intakeOn();
                        follower.followPath(grabPickup1_3, true);
                        setPathState(9);
                    }
                }
                break;
            case 9:

                if(!follower.isBusy()) {
                    if(timer.milliseconds()>1000) {
                        motors.setRampVelocityC(2050);
                        servos.turretGT(0.47);
                        timer.reset();
                        sleep(200);
                        follower.followPath(scorePickup1, true);
                        setPathState(10);
                    }
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

        servos.hoodMove(1);
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

    public void handleTurret() {

        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double thetaR = follower.getPose().getHeading();
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


        targetAngle = goalAngle - thetaR;
        targetAngle = Math.atan2(Math.sin(targetAngle), Math.cos(targetAngle));
        targetPosition = convertToNewRange(
                targetAngle,
                Math.PI / 2, -Math.PI / 2,
                0.2, 0.8
        );
        if(limelight.checkResults())
            targetPosition+=(limelight.getXPos()*0.0014);
        targetPosition = Math.min(Math.max(targetPosition,0.28),0.72);
        servos.turretGT(targetPosition);

    }
    public double convertToNewRange(double value,
                                    double oldMin, double oldMax,
                                    double newMin, double newMax) {
        return newMin + (value - oldMin) * (newMax - newMin) / (oldMax - oldMin);
    }

    static double getRPM(double d) {
        double r = 0.012 * d * d - 1.9 * d + 2070;
        r = Math.max(Math.min(r,3000),100);
        return r;
    }
    static double getHood(double d) {
        if (d < 130)
            return (d - 100) * (9.0 / 30.0);
        else if (d < 170)
            return 9 - (d - 130) * (1.0 / 40.0);
        else
            return 6 + (d - 170) * (5.0 / 150.0);
    }

    @Override
    public void stop() {
        //selectioner.stop();
    }


}