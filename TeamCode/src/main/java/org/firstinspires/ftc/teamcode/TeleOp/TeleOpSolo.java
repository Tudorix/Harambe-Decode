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
    import org.firstinspires.ftc.teamcode.Threads.Selectioner;
    import org.firstinspires.ftc.teamcode.Threads.Servos;
    import org.firstinspires.ftc.teamcode.Threads.Motors;
    import org.firstinspires.ftc.teamcode.Utils.Color;
    import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
    import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;

    import java.util.ArrayList;

    //comment


    @TeleOp(name="TeleOp Uno", group = "Solo")
    public class TeleOpSolo extends LinearOpMode {
        Servos servos = null;
        HardwareClass hardwareClass = null;
        Holonomic holonomic = null;
        Motors motors = null;
        Limelight limelight = null;
        Selectioner selectioner = null;
        boolean rampUp = false;

        double velocity;
        int target = -1;
        boolean override = false, reset;
        private Follower follower;
        double offset = 0;
        int greenPos = -1;
        double targetPosition, targetAngle;

        public static Pose startingPose = new Pose(HardwareClass.startX,HardwareClass.startY,Math.toRadians(HardwareClass.startAngle));
        ArrayList<Color> obeliskColors = new ArrayList<>();
        ElapsedTime time = new ElapsedTime();

        private TelemetryManager telemetryM;
        //private final ElapsedTime checkLimelight = new ElapsedTime();
        @Override
        public void runOpMode()  {
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
            ElapsedTime timer = new ElapsedTime();
            motors.setRampCoefs();
            //selectioner.setObeliskColors(obeliskColors);
            waitForStart();

            limelight.setPipeline(1);

            limelight.setup();
            limelight.start();
            follower.startTeleopDrive();

            if(!holonomic.getStatus()){
                holonomic.start();
            }
            servos.hoodMove(10);
            selectioner.aladam();
            selectioner.init();
            selectioner.start();


            while(opModeIsActive()) {
                if(time.seconds()>2)
                    reset = true;
                if(greenPos==-1)
                    greenPos = limelight.checkApriltagResults();
                if(greenPos!=-1 && limelight.getStatus()) {
                    limelight.stop();
                    target = 0;
                    limelight.setPipeline(4);
                }
                double distance = limelight.getDistanceOD(follower.getPose().getX(), follower.getPose().getY(),target);
                if(Math.abs(motors.getRampError((int)getRPM(distance)))-50<50 && !gamepad1.isRumbling())
                    gamepad1.rumble(150);
                telemetry.clearAll();
                velocity = motors.getVelocity();
                handleTurret();

                servos.hoodMove((int)getHood(distance));
                if(rampUp)motors.setRampVelocityC((int)getRPM(distance));

                if(gamepad1.right_stick_button){
                    target=0;
                    limelight.setPipeline(4);
                }

                if(gamepad1.left_stick_button){
                    target=1;
                    limelight.setPipeline(0);
                }

                if (gamepad1.right_trigger > 0.23 && !selectioner.ballsfull) {
                    motors.intakeOn();
                    selectioner.resetBrat();
                }
                else if(gamepad1.left_trigger>0.23 || selectioner.ballsfull && !reset) {
                    motors.intakeReverse();
                    time.reset();
                }
                else
                    motors.intakeOff();

                if (gamepad1.right_bumper) {
                    rampUp = true;
                }

                if (gamepad1.left_bumper) {
                    motors.rampStop();
                    rampUp = false;
                }

                if(gamepad1.ps)
                    follower.setPose(PoseStorage.resetPose);

                if(gamepad1.options) {
                    override = true;
                    motors.rampStop();
                }
                if(gamepad1.x) {
                    reset = false;

                   selectioner.leftServoUp();
                }
                if(gamepad1.b){
                    reset = false;
                    selectioner.rightServoUp();
                }
                if(gamepad1.a){
                    reset = false;
                    selectioner.topServoUp();
                }
                if(gamepad1.y) {
                    reset = false;
                    selectioner.unloadBalls();
                }

    //            if(gamepad1.dpad_left){
    //                selectioner.shootGreen();
    //            }
    //            if(gamepad1.dpad_right){
    //                selectioner.shootPurple();
    //            }
                if(gamepad1.dpad_up){
                    reset = false;
                    int pos = greenPos+1;
                    if(pos>=4)pos=pos%3;
                    selectioner.printesaDinDubai((pos));
                }
                if(gamepad1.dpad_right){
                    reset = false;
                    selectioner.printesaDinDubai(greenPos);
                }

                if(gamepad1.dpad_left){
                    reset = false;
                    int pos = greenPos+2;
                    if(pos>=4)pos=pos%3;
                    selectioner.printesaDinDubai((pos));
                }
                if(gamepad1.dpad_down){
                    selectioner.resetBrat();
                }
                if(gamepad1.share)
                    override = false;
                selectioner.telemetry();
                telemetry.addData("x:" , follower.getPose().getX());
                telemetry.addData("y:" , follower.getPose().getY());
                telemetry.addData("Heading",follower.getPose().getHeading());
                telemetry.addData("Velocity:",motors.getVelocity());
                telemetry.addData("Velocity error:",motors.getRampError(velocity));
                telemetry.addData("Green position",greenPos);
                if(target == 0){
                   telemetry.addLine("Target: RED");
               }
                else {
                   telemetry.addLine("Target: BLUE");
                }
                telemetry.addData("offset:",offset);
                telemetry.addData("Distance to target: ", distance);
                telemetry.addData("Target:",targetPosition);
                telemetry.addData("TargetAngle:",targetAngle);
                telemetry.addData("Tx:", limelight.getXPos());
                telemetry.addData("Robot Velocity:",follower.getVelocity().getMagnitude()*0.0254);
                if(override)
                    telemetry.addLine("!!!! SPEED OVERRIDE !!!!");
                follower.update();
                telemetryM.update();
                telemetry.update();
                if(isStopRequested()) {
                    selectioner.stop();
                }
                setObelisk(); //hoara is my passion
            }
            selectioner.stop();
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
                0.15, 0.85
        );
            if(limelight.checkResults())
               targetPosition+=(limelight.getXPos()*0.002);
            targetPosition = Math.min(Math.max(targetPosition,0.25),0.75);
            servos.turretGT(targetPosition);
        }


        void setObelisk() {
            if (greenPos <= 0) return;

            obeliskColors.clear();

            obeliskColors.add(greenPos == 1 ? Color.GREEN : Color.PURPLE);
            obeliskColors.add(greenPos == 2 ? Color.GREEN : Color.PURPLE);
            obeliskColors.add(greenPos == 3 ? Color.GREEN : Color.PURPLE);

            //selectioner.setObeliskColors(obeliskColors);
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
    }