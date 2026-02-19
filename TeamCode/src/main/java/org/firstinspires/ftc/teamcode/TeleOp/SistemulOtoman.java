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

    import org.firstinspires.ftc.teamcode.HardwareClass;
    import org.firstinspires.ftc.teamcode.Threads.Holonomic;
    import org.firstinspires.ftc.teamcode.Threads.Limelight;
    import org.firstinspires.ftc.teamcode.Threads.Motors;
    import org.firstinspires.ftc.teamcode.Threads.Selectioner;
    import org.firstinspires.ftc.teamcode.Threads.Servos;
    import org.firstinspires.ftc.teamcode.Threads.Turret;
    import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
    import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
    import org.opencv.core.Mat;

    //comment

    @Config
    @TeleOp(name="SistemulOtoman", group = "Solo")
    public class SistemulOtoman extends LinearOpMode {
        Servos servos = null;
        HardwareClass hardwareClass = null;
        Holonomic holonomic = null;
        Motors motors = null;
        Limelight limelight = null;
        Selectioner selectioner = null;
        Turret turret = null;

        ElapsedTime Timer = null;

        boolean override = false;
        double targetPosition, targetAngle;
        double target = -1;
        int color = 0;

        double threshold = 0.68; // To Adjust
        private Follower follower;
        private int delay_shoot = 50;

        double goalHeight = 0;
        double cameraHeight = 0;
        double cameraAngle = 0;
        double minAngleLL = 0;

        // ---- SHOOTER PARAMETERS ----
        double threshold_far = 0.95; // To Adjust
        double threshold_close = 0.68; // To Adjust
        double threshold_mid = 0.8; // To Adjust

        int kp_far = 120;
        int kp_close = 60;
        int kp = 60;
        private int delay_fast = 100;
        private int delay_slow = 400;

        private int delay_brat_slow = 250;
        private int delay_brat_fast = 200;

        double hood_offset_1 = 0.2;
        double hood_offset_2 = 0.09;
        double hood_offset_3 = 0.02;

        // ----

        int putere_far = 4000;
        int putere_close = 2500;
        int putere_mid = 2800;
        int putere;
        int putere_tras;

        // ---- Goal Location ----

        public static int RED_X = 120, RED_Y = 160;
        public static int BLUE_X = 0, BLUE_Y = 100;

        public int turret_max = 0;
        public int turret_min = -357;

        double adjust = 0;
        double lastTx = 0;
        private TelemetryManager telemetryM;

        private double InregisSpeed = 0;
        int test_case = 1;
        int goal = 0;

        String mode = "drive";
        @Override
        public void runOpMode()  {
            // ---- Follower Init Location ----
            follower = Constants.createFollower(hardwareMap);
            Pose pose = new Pose(PoseStorage.autoPose.getX(), PoseStorage.autoPose.getY(),PoseStorage.autoPose.getPose().getHeading());
            follower.setStartingPose(pose);
            telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

            // ---- System Config ----
            servos = Servos.getInstance(hardwareMap , telemetry);
            motors = Motors.getInstance(hardwareMap);
            hardwareClass = HardwareClass.getInstance(hardwareMap);
            holonomic = Holonomic.getInstance(hardwareMap , gamepad1, gamepad2);
            limelight = Limelight.getInstance(hardwareMap,telemetry);
            selectioner = Selectioner.getInstance(hardwareClass, telemetry);
            turret = Turret.getInstance(hardwareMap,telemetry);

            // ---- Setting Motors ----
            hardwareClass.FL.setDirection(DcMotorSimple.Direction.REVERSE);
            hardwareClass.BL.setDirection(DcMotorSimple.Direction.REVERSE);
            motors.setRampCoefs();

            Timer = new ElapsedTime();

            waitForStart();

            // ---- Setup SubSys ----
            limelight.setPipeline(4);

            selectioner.resetServos();

            if(!turret.getStatus()){
                    turret.setup();
            }
            //turret.resetMotor();

            if(!holonomic.getStatus()){
                holonomic.start();
            }

            double distance = -1;
            delay_shoot = delay_fast;
            HardwareClass.bratDelay = delay_brat_fast;

            turret.goToPosition(adjust);

            while(opModeIsActive()) {
                HardwareClass.redScoreX = RED_X;
                HardwareClass.redScoreY = RED_Y;
                HardwareClass.blueScoreX = BLUE_X;
                HardwareClass.blueScoreY = BLUE_Y;

                follower.update();
                distance = limelight.getDistanceOD(follower.getPose().getX(), follower.getPose().getY(),goal);

                if(distance < 200){
                    putere = putere_close;
                    threshold = threshold_close;
                    kp = kp_close;
                    putere_tras = putere_close;
                }else if(distance >= 200 && distance < 280){
                    putere = putere_mid;
                    threshold = threshold_mid;
                    kp = kp_close;
                    putere_tras = putere_mid;
                }else if(distance >= 280){
                    putere = putere_far;
                    threshold = threshold_far;
                    kp = kp_far;
                    putere_tras = putere_far - 1000;
                }

                telemetry.addData("Distance: " , distance);
                telemetry.addData("Pose: " , follower.getPose());
                telemetry.addData("Putere: " , putere);
                telemetry.update();

                switch(mode){
                    case "drive":{

                        // ---- SHOOTING CLOSE ----
                        if(gamepad2.right_bumper){
                            motors.setCoefsMan(kp,0,0,1.2);
                            mode = "shoot_fast";
                        }

                        // ---- SHOOTING MODE ----

                        if(gamepad2.share){
                            delay_shoot = delay_slow;
                            HardwareClass.bratDelay = delay_brat_slow;
                        }

                        if(gamepad2.options){
                            delay_shoot = delay_fast;
                            HardwareClass.bratDelay = delay_brat_fast;
                        }

                        // ---- TURRET ----

                        if(gamepad2.y){
                            adjust = (int)((turret_max + turret_min) / 2);
                            target = -1;
                            turret.goToPosition(adjust);
                        }

                        if (gamepad2.dpad_left){
                            target = -1;
                            adjust += 40;
                            turret.goToPosition(adjust);
                        }

                        if(gamepad2.dpad_down){
                            turret.resetMotor();
                            target = -1;
                        }

                        if(gamepad2.dpad_right){
                            target = 1;
                        }

                        if (gamepad2.left_trigger > 0){
                            Pose pose1 = new Pose(72,72,90);
                            follower.setPose(pose1);
                        }

                       if(gamepad2.b){
                           goal = 0;
                           target = 1;
                       }

                        if(gamepad2.x){
                            goal = 2;
                            target = 1;
                        }

                       if(target == 1){
                           handleTurret();
                       }


                        //----- INTAKE ------
                        if(gamepad1.right_trigger > 0){
                            motors.intakeOn();
                        }
                        else if(gamepad1.left_trigger > 0){
                            motors.intakeReverse();
                        }else{
                            motors.intakeOff();
                        }

                        break;
                    }
                    case "shoot_fast":{
                        if(target == 1){
                            handleLL();
                        }

                        motors.setRampVelocityC(putere);
                        motors.intakeOn();

                        if (motors.getVelocity() > (int)(threshold * putere_tras)){
                            if(threshold <= 0.8){
                                shoot_short();
                            }else{
                                shoot_long();
                            }
                            sleep(200);
                            motors.setCoefsMan(20,0,0,1.2);
                            motors.setRampVelocityC(1000);
                            selectioner.resetServos();
                            motors.intakeOff();
                            mode = "drive";
                        }

                        if (gamepad1.dpad_right){
                            target = -1;
                            adjust += 2;
                            turret.goToPosition(adjust);
                        }

                        if (gamepad1.dpad_left){
                            target = -1;
                            adjust -= 2;
                            turret.goToPosition(adjust);
                        }

                        if(gamepad1.dpad_up){
                            target = 1;
                        }

                        if(gamepad1.right_trigger > 0){
                            motors.setCoefsMan(20,0,0,1.2);
                            motors.setRampVelocityC(1000);
                            selectioner.resetServos();
                            motors.intakeOff();
                            mode = "drive";
                        }
                        break;
                    }
                }
            }
        }

        // HandleTurret cu CAMERA
        public void handleLL(){
            try{
                if(limelight.checkResults()){
                    double tx = limelight.getXPos();
                    if(Math.abs(tx) > 2){
                        adjust -= (int)(tx * 0.4);
                        lastTx = tx;
                    }
                }else{
                    if(lastTx < 0){
                        adjust += 5;
                    }else{
                        adjust -= 5;
                    }
                }
                if(adjust < turret_min) adjust = turret_min;
                if(adjust > turret_max) adjust = turret_max;
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

            if(goal == 2) {
                limelight.setPipeline(0);
                dx = HardwareClass.blueScoreX - x;
                dy = HardwareClass.blueScoreY - y;
            }

            if(goal == 0) {
                limelight.setPipeline(4);
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
                    turret_min, turret_max
            );

            targetPosition = Math.min(Math.max(targetPosition,turret_min),turret_max);
            adjust = targetPosition;
            turret.goToPosition(targetPosition);
        }


        // Distanta cu TY
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
            r = Math.max(Math.min(r,3000),100);
            return (int)r;
        }

        void shoot_short(){
            //Shoot First
            HoodToPos(hood_offset_1);
            selectioner.rightServoUp();
            sleep(delay_shoot);
            //Shoot Second
            HoodToPos(hood_offset_2);
            selectioner.leftServoUp();
            sleep(delay_shoot);
            //Shoot Third
            HoodToPos(hood_offset_3);
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