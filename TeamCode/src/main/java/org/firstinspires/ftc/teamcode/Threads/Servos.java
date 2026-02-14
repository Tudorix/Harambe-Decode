package org.firstinspires.ftc.teamcode.Threads;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareClass;

import java.util.concurrent.ForkJoinPool;

public class Servos {
    private HardwareClass hardwareClass;
    private Servo angleServo, turret1, turret2;
    private static Servos single_instance = null;
    private double medianError = -1;

    public Servos(HardwareClass hardwareClass, Telemetry telemetry , HardwareMap hardwareMap){
        this.hardwareClass = hardwareClass;
        this.angleServo = hardwareClass.angle;
        //this.turret1 = hardwareClass.turret1;
        //this.turret2 = hardwareClass.turret2;
        //turret1.setDirection(Servo.Direction.FORWARD);
        //turret2.setDirection(Servo.Direction.FORWARD);
    }

    public void turretGT(double pos){
        turret1.setPosition(pos);
        turret2.setPosition(pos);
    }

    public void hoodMove(int angle, double error) { // Este o tentativa pentru PIDF tunner, ar trebui sa scada usor unchiul daca turatia motorului este prea mica pentru a trage. NU ESTE BINE CALIBRATA!
        if (angle > 24) angle = 24;
        if (angle < 0) angle = 0;


        double deltaAngle = ((double) angle / 24.0) * -0.5 + 0.89;
        error = Math.max(Math.min(error, 120), 0);
        if (medianError < 0) {
            medianError = error;
        }
        else if(Math.abs(medianError-error)>60){
            medianError = error;
        }
        angleServo.setPosition(deltaAngle-medianError*0.0003); // cam mult dar e ok :)
    }

    public void hoodMove(int angle){
        if(angle>24)angle = 24;
        if(angle<0)angle = 0;
        double deltaAngle = ((double)angle/24.0)*-0.5+0.85;
        angleServo.setPosition(deltaAngle);
    }

    public void sleep(int sec){
        try {
            Thread.sleep(sec);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public static synchronized Servos getInstance(HardwareMap hardwareMap , Telemetry telemetry ){
        if(single_instance == null){
            single_instance = new Servos(HardwareClass.getInstance(hardwareMap), telemetry , hardwareMap);
        }
        return single_instance;
    }
}
