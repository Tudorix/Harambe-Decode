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

        double threshold = 0.75;
        private Follower follower;
        private TelemetryManager telemetryM;
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

            hardwareClass.FL.setDirection(DcMotorSimple.Direction.REVERSE);
            hardwareClass.BL.setDirection(DcMotorSimple.Direction.REVERSE);
            motors.setRampCoefs();

            waitForStart();

            limelight.setPipeline(1);
            limelight.setup();
            limelight.start();

            if(!holonomic.getStatus()){
                holonomic.start();
            }

            selectioner.init();
            selectioner.start();

            double distance = -1;
            while(opModeIsActive()) {
                if(gamepad1.left_bumper){
                    distance = limelight.getDistanceOD(follower.getPose().getX(),follower.getPose().getY(),0);
                    motors.setCoefsMan(20,0,0,1.2);
                    motors.setRampVelocityC((int)(0.33 * getRPM(distance)));
                }

                if(gamepad1.right_bumper){
                    distance = limelight.getDistanceOD(follower.getPose().getX(),follower.getPose().getY(),0);
                    motors.setCoefsMan(60,0,0,1.2);
                    motors.setRampVelocityC((int)(getRPM(distance)));// * 1.1
                }

                if (motors.getVelocity() > (int)(threshold * getRPM(distance))){
                    selectioner.printesaDinDubai(-1);
                }
            }
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