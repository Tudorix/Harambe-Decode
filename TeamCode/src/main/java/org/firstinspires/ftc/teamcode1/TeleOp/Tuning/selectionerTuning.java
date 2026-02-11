package org.firstinspires.ftc.teamcode.TeleOp.Tuning;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import org.firstinspires.ftc.teamcode.Threads.Selectioner;
import org.firstinspires.ftc.teamcode.HardwareClass;
@TeleOp
@Config
public class selectionerTuning extends OpMode {

    HardwareClass hardwareClass;
    Selectioner selectioner;
    boolean running = true;

    @Override
    public void init() {
        hardwareClass = HardwareClass.getInstance(hardwareMap);
        selectioner = Selectioner.getInstance(hardwareClass, telemetry);
        selectioner.start();
    }

    @Override
    public void loop() {
        while (running) {
            telemetry.clear();
            telemetry.update();
            if(gamepad1.a)
                selectioner.shootPurple();
            if(gamepad1.b)
                selectioner.shootGreen();
            if(gamepad1.ps)
                selectioner.purge();
            if(gamepad1.options)
                running=false;
        }
    selectioner.stop();
    }
}