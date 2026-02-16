    package org.firstinspires.ftc.teamcode.TeleOp;

    import com.acmerobotics.dashboard.config.Config;
    import com.bylazar.telemetry.PanelsTelemetry;
    import com.bylazar.telemetry.TelemetryManager;
    import com.pedropathing.follower.Follower;
    import com.pedropathing.geometry.Pose;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.util.ElapsedTime;

    import org.firstinspires.ftc.robotcore.external.Telemetry;
    import org.firstinspires.ftc.teamcode.HardwareClass;
    import org.firstinspires.ftc.teamcode.Threads.Holonomic;
    import org.firstinspires.ftc.teamcode.Threads.Limelight;
    import org.firstinspires.ftc.teamcode.Threads.Motors;
    import org.firstinspires.ftc.teamcode.Threads.Selectioner;
    import org.firstinspires.ftc.teamcode.Threads.Servos;
    import org.firstinspires.ftc.teamcode.Threads.Turret;
    import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
    import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;

    //comment

    @Config
    @TeleOp(name="Soare", group = "Solo")
    public class RotatieDupaSoare extends LinearOpMode {
        Servos servos = null;
        HardwareClass hardwareClass = null;
        Holonomic holonomic = null;
        Motors motors = null;
        Limelight limelight = null;
        Selectioner selectioner = null;
        Turret turret = null;

        ElapsedTime Timer = null;

        public static int RED_X = 135;
        public static int RED_Y = 135;

        boolean override = false;
        double targetPosition, targetAngle;
        double target = 0;

        double threshold = 0.68; // To Adjust
        double threshold_far = 1.1; // To Adjust
        double threshold_close = 0.69; // To Adjust
        private Follower follower;
        private int delay_shoot = 300;
        private TelemetryManager telemetryM;

        double goalHeight = 0;
        double cameraHeight = 0;
        double cameraAngle = 0;
        double minAngleLL = 0;
        double adjust = 0;

        private double InregisSpeed = 0;
        int test_case = 1;

        String mode = "drive";
        @Override
        public void runOpMode()  {
            //Init phase
            follower = Constants.createFollower(hardwareMap);
            Pose pose = new Pose(PoseStorage.autoPose.getX() - 72, PoseStorage.autoPose.getY() - 72,PoseStorage.autoPose.getPose().getHeading());
            follower.setStartingPose(pose);

            telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

            servos = Servos.getInstance(hardwareMap , telemetry);
            motors = Motors.getInstance(hardwareMap);
            hardwareClass = HardwareClass.getInstance(hardwareMap);
            holonomic = Holonomic.getInstance(hardwareMap , gamepad1, gamepad2);
            limelight = Limelight.getInstance(hardwareMap,telemetry);
            selectioner = Selectioner.getInstance(hardwareClass, telemetry);
            turret = Turret.getInstance(hardwareMap,telemetry);

            hardwareClass.FL.setDirection(DcMotorSimple.Direction.REVERSE);
            hardwareClass.BL.setDirection(DcMotorSimple.Direction.REVERSE);
            motors.setRampCoefs();

            Timer = new ElapsedTime();
            follower.startTeleopDrive();

            waitForStart();

            limelight.setPipeline(1);
            limelight.setup();
            limelight.start();

            if(!holonomic.getStatus()){
                holonomic.start();
            }

            if(!turret.getStatus()){
                turret.setup();
            }

            double distance = -1;
            while(opModeIsActive()) {
                follower.update();
                switch(mode){
                    case "drive":{
                        telemetry.addData("Distance: " , getDistance());
                        telemetry.update();
                        handleTurret();
                        break;
                    }
                }
            }
        }

        public void handleTurret(){
            try{
                double x = limelight.getXPos();
                if(Math.abs(x) < 2){
                    adjust += (int)(x / 2);
                }
                if(adjust < -150) adjust = -150;
                if(adjust > 200) adjust = 200;
                turret.goToPosition(adjust);
            }catch (Exception ex){
                adjust = 0;
                turret.goToPosition(adjust);
            }
        }

        public double getDistance(){
            try{
                double Ty = limelight.getYPos();
                double dh = goalHeight - cameraHeight;
                double thetaFin = Math.abs(minAngleLL - cameraAngle);
                double du = Math.tan(Math.abs(Ty - thetaFin));
                return  dh / du;
            }catch (Exception ex){
                return 215;
            }
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

        void shoot_short(){
            //Shoot First
            HoodToPos(0);
            selectioner.rightServoUp();
            sleep(delay_shoot);
            //Shoot Second
            HoodToPos(0.02);
            selectioner.leftServoUp();
            sleep(delay_shoot);
            //Shoot Third
            HoodToPos(0.07);
            selectioner.topServoUp();
            HoodToPos(0);
        }

        void HoodToPos(double pos){
            hardwareClass.angle.setPosition(hardwareClass.hood_down - pos);
        }

        void shoot_long(){
            sleep(delay_shoot);
            selectioner.rightServoUp();
            sleep(delay_shoot);
            selectioner.leftServoUp();
            sleep(delay_shoot);
            selectioner.topServoUp();
        }

        void shoot_one(){
            selectioner.topServoUp();
        }


    }