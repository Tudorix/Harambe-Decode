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
    import org.firstinspires.ftc.teamcode.Threads.Localization;
    import org.firstinspires.ftc.teamcode.Threads.Motors;
    import org.firstinspires.ftc.teamcode.Threads.Selectioner;
    import org.firstinspires.ftc.teamcode.Threads.Servos;
    import org.firstinspires.ftc.teamcode.Utils.Color;
    import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
    import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;

    import java.util.ArrayList;

    //comment


    @TeleOp(name="TeleOp Uno", group = "Solo")
    public class TeleOpDual extends LinearOpMode {
        Servos servos = null;
        HardwareClass hardwareClass = null;
        Holonomic holonomic = null;
        Motors motors = null;
        Localization localization = null;
        Selectioner selectioner = null;
        private Follower follower;

        public static Pose startingPose = new Pose(HardwareClass.startX,HardwareClass.startY,Math.toRadians(HardwareClass.startAngle));
        ArrayList<Color> obeliskColors = new ArrayList<>();
        ElapsedTime time = new ElapsedTime();

        private TelemetryManager telemetryM;
        //private final ElapsedTime checkLimelight = new ElapsedTime();
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
            localization = Localization.getInstance(hardwareMap,telemetry);
            selectioner = Selectioner.getInstance(hardwareClass, telemetry);

            hardwareClass.FL.setDirection(DcMotorSimple.Direction.REVERSE);
            hardwareClass.BL.setDirection(DcMotorSimple.Direction.REVERSE);
            ElapsedTime timer = new ElapsedTime();
            motors.setRampCoefs();
            //selectioner.setObeliskColors(obeliskColors[0]);

            waitForStart();

            localization.setPipeline(1);
            localization.setup();
            localization.start();
            //follower.startTeleopDrive();

            if(!holonomic.getStatus()){
                holonomic.start();
            }

            servos.hoodMove(10);
            selectioner.aladam();
            selectioner.init();
            selectioner.start();


            while(opModeIsActive()) {

            }
        }
    }