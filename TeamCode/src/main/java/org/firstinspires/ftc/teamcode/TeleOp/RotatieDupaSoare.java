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

        public static int RED_X = 90, RED_Y = 100;
        public static int BLUE_X = 70, BLUE_Y = 0;

        boolean override = false;
        double targetPosition, targetAngle;
        double target = 0;

        double threshold = 0.68; // To Adjust
        double threshold_far = 1.1; // To Adjust
        double threshold_close = 0.69; // To Adjust
        private Follower follower;
        private int delay_shoot = 300;
        private TelemetryManager telemetryM;

        double goalHeight = 74;
        double cameraHeight = 29;
        double cameraAngle = 20;
        double minAngleLL = 21;
        double adjust = 0;

        private double InregisSpeed = 0;
        int test_case = 1;

        double P_GAIN = 0,D_GAIN = 0, lastTx, turretAngleDeg, frequency = 50;

        String mode = "drive";
        @Override
        public void runOpMode()  {
            //Init phase
            follower = Constants.createFollower(hardwareMap);
            Pose pose = new Pose(72, 72,90);
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

            //limelight.setup();
            //limelight.start();
            limelight.setPipeline(4);

            if(!holonomic.getStatus()){
                holonomic.start();
            }

            HardwareClass.redScoreX = RED_X;
            HardwareClass.redScoreY = RED_Y;
            HardwareClass.blueScoreX = BLUE_X;
            HardwareClass.blueScoreY = BLUE_Y;

            if(!turret.getStatus()){
                turret.setup();
                turret.resetMotor();
            }

            int mod = 0;

            double distance = -1;
            while(opModeIsActive()) {
                follower.update();
                switch(mode){
                    case "drive":{
                        HardwareClass.redScoreX = RED_X;
                        HardwareClass.redScoreY = RED_Y;
                        HardwareClass.blueScoreX = BLUE_X;
                        HardwareClass.blueScoreY = BLUE_Y;
                        telemetry.addData("Pose" , follower.getPose());
                        telemetry.update();

                        if(gamepad1.a){
                            mod = 0;
                        }

                        if(gamepad1.b){
                            mod = 1;
                        }
                        if(mod == 0){
                            handleTurret();
                        }

                        if(mod == 1){
                            handleLL();
                        }

                        break;
                    }
                }
            }
        }

        public void handleLL(){
            try{
                if(limelight.checkResults()){
                    double tx = limelight.getXPos();
                    if(Math.abs(tx) > 2){
                        adjust -= (int)(tx * 0.45);
                        lastTx = tx;
                    }
                }else{
                    if(lastTx < 0){
                        adjust += 5;
                    }else{
                        adjust -= 5;
                    }
                }
                if(adjust < -150) adjust = -150;
                if(adjust > 200) adjust = 200;
                turret.goToPosition(adjust);

            }catch (Exception ex){
                adjust = 0;
                turret.goToPosition(adjust);
            }
        }

        public void handleTurret() {
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
                    -150, 200
            );

            targetPosition = Math.min(Math.max(targetPosition,-150),200);
            adjust = targetPosition;
            turret.goToPosition(targetPosition);
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

        public double checkAprilTagPosPD() { // nu stiu daca merge, trebuie calibrate valorile
            if (limelight.checkResults()) {
                double tx = limelight.getXPos();
                if (Math.abs(tx) < 2) {
                    tx = 0;
                }
                double p = P_GAIN * tx;
                double d = D_GAIN * (tx - lastTx) / (1.0/frequency);
                lastTx = tx;
                double deltaDeg = -(p + d);
                turretAngleDeg += deltaDeg;
                turretAngleDeg = Math.max(-150, Math.min(200, turretAngleDeg));   //check clamping !!!
            }
            return convertToNewRange(turretAngleDeg,-50,50,-150,200);   //check min/max !!!
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