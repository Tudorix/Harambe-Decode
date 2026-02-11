    package org.firstinspires.ftc.teamcode.TeleOp;

    import com.bylazar.telemetry.PanelsTelemetry;
    import com.bylazar.telemetry.TelemetryManager;
    import com.pedropathing.follower.Follower;
    import com.pedropathing.geometry.Pose;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.util.ElapsedTime;

    import org.firstinspires.ftc.teamcode.HardwareClass;
    import org.firstinspires.ftc.teamcode.Threads.Holonomic;
    import org.firstinspires.ftc.teamcode.Threads.Limelight;
    import org.firstinspires.ftc.teamcode.Threads.Motors;
    import org.firstinspires.ftc.teamcode.Threads.Selectioner;
    import org.firstinspires.ftc.teamcode.Threads.Servos;
    import org.firstinspires.ftc.teamcode.Threads.Turret;
    import org.firstinspires.ftc.teamcode.Utils.Color;
    import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
    import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;

    import java.util.ArrayList;

    //comment


    @TeleOp(name="TeleOp Pisica din dubai", group = "Solo")
    public class TeleOpDual extends LinearOpMode {
        Servos servos = null;
        HardwareClass hardwareClass = null;
        Holonomic holonomic = null;
        Motors motors = null;
        Limelight limelight = null;
        Selectioner selectioner = null;
        Turret turret = null;

        boolean override = false;
        double targetPosition, targetAngle, target;

        double threshold = 0.70; // To Adjust
        private Follower follower;
        private TelemetryManager telemetryM;

        String mode = "drive";
        @Override
        public void runOpMode()  {
            //Init phase
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(PoseStorage.autoPose.getX()-72, PoseStorage.autoPose.getY()-72,PoseStorage.autoPose.getPose().getHeading()));

            telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

            servos = Servos.getInstance(hardwareMap , telemetry);
            motors = Motors.getInstance(hardwareMap);
            hardwareClass = HardwareClass.getInstance(hardwareMap);
            holonomic = Holonomic.getInstance(hardwareMap , gamepad1, gamepad2);
            limelight = Limelight.getInstance(hardwareMap,telemetry);
            selectioner = Selectioner.getInstance(hardwareClass, telemetry);
            //turret = Turret.getInstance(hardwareMap,telemetry);

            hardwareClass.FL.setDirection(DcMotorSimple.Direction.REVERSE);
            hardwareClass.BL.setDirection(DcMotorSimple.Direction.REVERSE);
            motors.setRampCoefs();

            waitForStart();

            /*
            if(!turret.getStatus()){
                turret.setup();
            }
            */


            limelight.setPipeline(1);
            limelight.setup();
            limelight.start();

            if(!holonomic.getStatus()){
                holonomic.start();
            }

            double distance = -1;
            while(opModeIsActive()) {
                switch(mode){
                    case "drive":{
                        if(gamepad1.left_bumper){
                            distance = limelight.getDistanceOD(follower.getPose().getX(),follower.getPose().getY(),0);
                            motors.setCoefsMan(20,0,0,1.2);
                            motors.setRampVelocityC((int)(0.33 * getRPM(distance)));
                        }

                        if(gamepad1.right_bumper){
                            distance = limelight.getDistanceOD(follower.getPose().getX(),follower.getPose().getY(),0);
                            motors.setCoefsMan(60,0,0,1.2);
                            motors.setRampVelocityC((int)(getRPM(distance)));// * 1.1
                            mode = "shoot";
                        }

                        if(gamepad1.right_trigger > 0){
                            motors.intakeOn();
                        }
                        else if(gamepad1.right_trigger <= 0){
                            motors.intakeOff();
                        }

                        break;
                    }
                    case "shoot":{
                        if (motors.getVelocity() > (int)(threshold * getRPM(distance))){
                            selectioner.unloadBallsQuick();
                            distance = limelight.getDistanceOD(follower.getPose().getX(),follower.getPose().getY(),0);
                            motors.setCoefsMan(20,0,0,1.2);
                            motors.setRampVelocityC((int)(0.33 * getRPM(distance)));
                            selectioner.resetServos();
                            mode = "drive";
                        }
                        break;
                    }
                }
            }
        }

        public void handleTurret() {
            if(override){
                servos.turretGT(0.5);
                return;
            }
            double x = follower.getPose().getX();
            double y = follower.getPose().getY();
            double dx=0,dy=0;


            if(target == -1){
                dx = -72 - x;
                dy = 0 - y;
            }
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
                    -Math.PI / 2, Math.PI / 2,
                    0.15, 0.85 // New Encoder Ticks
            );

            targetPosition = Math.min(Math.max(targetPosition,0.25),0.75); // Encoder Positions
            turret.goToPosition(targetPosition);
        }

        public double convertToNewRange(double value,
                                        double oldMin, double oldMax,
                                        double newMin, double newMax) {
            return newMin + (value - oldMin) * (newMax - newMin) / (oldMax - oldMin);
        }

        static int getRPM(double d) {
            double r = 0.012 * d * d - 1.9 * d + 2070;
            r = Math.max(Math.min(r,3000),100);
            return (int)r;
        }

        static double getHood(double d) {
            if (d < 130)
                return (d - 100) * (9.0 / 30.0);
            else if (d < 170)
                return 9 - (d - 130) * (1.0 / 40.0);
            else
                return 6 + (d - 170) * (5.0 / 150.0);
        }
    }