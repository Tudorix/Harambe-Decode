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


    @TeleOp(name="TeleOp Uno", group = "Solo")
    public class TeleOpDual extends LinearOpMode {
        Servos servos = null;
        HardwareClass hardwareClass = null;
        Holonomic holonomic = null;
        Motors motors = null;
        Limelight limelight = null;
        Selectioner selectioner = null;
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


            while(opModeIsActive()) {

            }
        }
    }