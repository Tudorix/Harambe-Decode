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
    import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
    import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;

    //comment


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
        double target = 0;
        int color = 0;

        double threshold = 0.68; // To Adjust
        private Follower follower;
        private int delay_shoot = 50;

        // ---- SHOOTER PARAMETERS ----
        double threshold_far = 1.1; // To Adjust
        double threshold_close = 0.68; // To Adjust

        int kp_far = 120;
        int kp_close = 60;
        private int delay_fast = 50;
        private int delay_slow = 400;

        private int delay_brat_slow = 250;
        private int delay_brat_fast = 150;

        double hood_offset_1 = 0;
        double hood_offset_2 = 0.04;
        double hood_offset_3 = 0.08;

        // ----

        int adjust = 0;
        private TelemetryManager telemetryM;

        private double InregisSpeed = 0;
        int test_case = 1;

        String mode = "drive";
        @Override
        public void runOpMode()  {
            // ---- Follower Init Location ----
            follower = Constants.createFollower(hardwareMap);
            //Pose pose = new Pose(PoseStorage.autoPose.getX()-72, PoseStorage.autoPose.getY()-72,PoseStorage.autoPose.getPose().getHeading());
            Pose pose = new Pose(144 - PoseStorage.autoPose.getX(), PoseStorage.autoPose.getY(),PoseStorage.autoPose.getPose().getHeading());
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
            limelight.setPipeline(1);
            limelight.setup();
            limelight.start();

            selectioner.resetServos();

            if(!turret.getStatus()){
                turret.setup();
                turret.resetMotor();
            }

            if(!holonomic.getStatus()){
                holonomic.start();
            }

            double distance = -1;
            delay_shoot = delay_fast;
            HardwareClass.bratDelay = delay_brat_fast;

            while(opModeIsActive()) {
                follower.update();
                switch(mode){
                    case "drive":{

                        // ---- CONST SPEEEEEED ----
                        if(gamepad1.dpad_down){
                            motors.setCoefsMan(20,0,0,1.2);
                            motors.setRampVelocityC(1000);
                        }

                        // ---- SHOOTING CLOSE ----
                        if(gamepad1.right_bumper){
                            distance = limelight.getDistanceOD(follower.getPose().getX(),follower.getPose().getY(),color);
                            motors.setCoefsMan(kp_close,0,0,1.2);
                            motors.setRampVelocityC((int)(getRPM(distance)));// * 1.1
                            threshold = threshold_close;
                            mode = "shoot_fast";
                        }

                        // ---- SHOOTING FAR ----
                        if(gamepad1.left_bumper){
                            distance = limelight.getDistanceOD(follower.getPose().getX(),follower.getPose().getY(),color);
                            motors.setCoefsMan(kp_far,0,0,1.2);
                            motors.setRampVelocityC((int)(getRPM(distance)) + 1000);// * 1.1
                            threshold = threshold_far;
                            mode = "shoot_fast";
                        }

                        // ---- SHOOTING MODE ----

                        if(gamepad1.share){
                            delay_shoot = delay_slow;
                            HardwareClass.bratDelay = delay_brat_slow;
                        }

                        if(gamepad1.options){
                            delay_shoot = delay_fast;
                            HardwareClass.bratDelay = delay_brat_fast;
                        }

                        // ---- TURRET ----

                        if (gamepad1.dpad_right){
                            adjust += 2;
                        }

                        if (gamepad1.dpad_left){
                            adjust -= 2;
                        }

                        if(gamepad1.dpad_up){
                            follower.setPose(new Pose(72,72,180));
                        }

                        if(gamepad1.y){
                            target = -1;
                        }

                        if(gamepad1.b){
                            target = 0;
                            color = 0;
                        }

                        if(gamepad1.x){
                            target = 1;
                            color = 1;
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

                        telemetry.addData("Pose: " , follower.getPose());
                        telemetry.update();

                        break;
                    }
                    case "shoot_fast":{
                        handleTurret();
                        distance = limelight.getDistanceOD(follower.getPose().getX(),follower.getPose().getY(),color);
                        motors.intakeOn();
                        if (motors.getVelocity() > (int)(threshold * getRPM(distance))){
                            if(threshold < 0.8){
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

                        if(gamepad1.right_trigger > 0){
                            motors.setCoefsMan(20,0,0,1.2);
                            motors.setRampVelocityC(1000);
                            selectioner.resetServos();
                            motors.intakeOff();
                            mode = "drive";
                        }

                        telemetry.addData("Pose: " , follower.getPose());
                        telemetry.update();
                        break;
                    }
                }
            }
        }

        public void handleTurret() {
            if (target == -1){
                turret.goToPosition(adjust);
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
                    -200, 200 // New Encoder Ticks
            );

            if(targetPosition > 200){
                targetPosition = 200;
            }

            if(targetPosition < -200){
                targetPosition = -200;
            }

            //targetPosition = Math.min(Math.max(targetPosition,0.25),0.75);
            telemetry.addData("Target Pos" , targetPosition);
            telemetry.update();
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