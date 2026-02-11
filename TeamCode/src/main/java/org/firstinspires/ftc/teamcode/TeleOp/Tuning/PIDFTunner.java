package org.firstinspires.ftc.teamcode.TeleOp.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClass;
import org.firstinspires.ftc.teamcode.Threads.Servos;
import org.firstinspires.ftc.teamcode.Threads.Motors;
//requires motor class!!!!!
//I will change the tuner in future update!!

@Config
@TeleOp(name="PIDF Tunner", group = "Solo")
public class PIDFTunner extends LinearOpMode {
    Servos servos = null;
    HardwareClass hardwareClass = null;
    Motors motors = null;

    private TelemetryManager telemetryM;
    @Override
    public void runOpMode()  {

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        servos = Servos.getInstance(hardwareMap , telemetry);
        motors = Motors.getInstance(hardwareMap);
        hardwareClass = HardwareClass.getInstance(hardwareMap);
        hardwareClass.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareClass.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareClass.ramp.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();
         ElapsedTime delay = new ElapsedTime();
        double coefs[] = { 35, 0 , 0 , 2.5 };
        int target = 0;
        double size = 0.1;
        double targetVelocity = 3000;
        int currentAngle = 10;
        hardwareClass.ramp.setDirection(DcMotorSimple.Direction.FORWARD);
        //                 P   I   D   F
        while(opModeIsActive()) {
            double velocity = motors.getVelocity();
            double error = targetVelocity - velocity;
            servos.hoodMove(currentAngle,error);
            if (gamepad1.ps) {
                telemetry.clear();
                motors.setCoefsMan(coefs[0],coefs[1],coefs[2],coefs[3]);
                telemetry.addLine("Coefs set!");
                telemetry.update();
                sleep(750);
            }

            if (gamepad1.right_bumper) {
                motors.setRampVelocityC((int)targetVelocity);
            }

            if (gamepad1.left_bumper) {
                motors.rampStop();
            }

            if (gamepad1.dpad_right) {
                if (target < coefs.length - 1 && delay.milliseconds() > 300){
                    target++;
                    delay.reset();
                }
            }
            if (gamepad1.dpad_left) {
                if (target > 0 && delay.milliseconds()>300) {
                    target--;
                    delay.reset();
                }
            }

            if (gamepad1.dpad_up) {
                if (delay.milliseconds() > 200) {
                    coefs[target] += size;
                    delay.reset();
                }
            }
            if (gamepad1.dpad_down) {
                if (delay.milliseconds() > 200) {
                    coefs[target] -= size;
                    delay.reset();
                }
            }

            if (gamepad1.right_trigger > 0.3) {
                if (delay.milliseconds() > 200) {
                    targetVelocity += 50;
                    delay.reset();
                }
            }
            if (gamepad1.left_trigger > 0.3) {
                if (delay.milliseconds() > 200) {
                    targetVelocity -= 50;
                    delay.reset();
                }
            }
            if(gamepad1.options){
                if(delay.milliseconds()>200){
                    currentAngle+=1;
                    delay.reset();
                }
            }
            if(gamepad1.share){
                if(delay.milliseconds()>200) {
                    currentAngle -= 1;
                    delay.reset();
                }
            }



            telemetry.clear();
            telemetry.addData("P: ", coefs[0]);
            telemetry.addData("I: ", coefs[1]);
            telemetry.addData("D: ", coefs[2]);
            telemetry.addData("F: ", coefs[3]);
            telemetry.addData("Target:", target);
            telemetry.addData("Error:", error);
            telemetry.addData("Angle ",currentAngle);
            telemetry.update();
            telemetryM.addData("Velocity:",velocity);
            telemetryM.update();
        }
    }
}