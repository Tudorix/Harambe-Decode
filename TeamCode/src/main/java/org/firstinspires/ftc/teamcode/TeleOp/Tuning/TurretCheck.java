package org.firstinspires.ftc.teamcode.TeleOp.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Threads.Limelight;
import org.firstinspires.ftc.teamcode.Threads.Turret;
import org.firstinspires.ftc.teamcode.Threads.TurretPID;

@TeleOp
@Config
public class TurretCheck extends OpMode {

    public DcMotorEx motor = null;
    public static String motorName = "TR";

    public static double kp = 0.01, ki = 0, kd = 0.005;
    public static double target = 0;

    Turret turret = null;
    public static double Position = 0;

    @Override
    public void init() {
        this.motor = hardwareMap.get(DcMotorEx.class, motorName);
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //turret = Turret.getInstance(hardwareMap,telemetry);
        //turret.setup();
        //turret.resetMotor();
    }

    @Override
    public void loop() {

        //turret.setPID(kp,ki,kd);
        //turret.goToPosition(target);
        telemetry.addData("Position" , motor.getCurrentPosition());
        telemetry.update();
    }
}