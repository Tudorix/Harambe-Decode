package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

@Config
public class HardwareClass {


    //Declarations
    public DcMotor FR, FL , BR , BL;

    public DcMotorEx ramp, ramp2, turret; public DcMotor intakeMotor;

    private static HardwareClass hardwareClass = null;


    public Servo angle;
    public Servo selectTop, selectBotR, selectBotL,turret1,turret2;
    public RevColorSensorV3 colorTop, colorBotR, colorBotL; //selection sensors
    public RevColorSensorV3 colorTop2, colorBotR2, colorBotL2;
    public static double startX = 64 ,startY = 0, startAngle=180;//Inch
    public static double redX = -57, redY = 57;
    public static double redScoreX = -65, redScoreY = 65;       //65   62
    public static double blueX = -57, blueY = -57;
    public static double blueScoreX = -65, blueScoreY = -65;

    public static int bratDelay = 250,bratDownDelay = 150; // milisecunde

    public static  double selectTopLOW = 0.86, selectBotLLOW = 0.9,selectBotRLOW =0.95 ;//0.14

    public static  double selectTopREST = 0.7, selectBotLREST = 0.74,selectBotRREST =0.80 ;//0.35 //Intermediate to make sure it reads color correctly         calibrate!

    public static  double selectTopHIGH = 0.45, selectBotLHIGH = 0.5,selectBotRHIGH = 0.54;//0.57



    public HardwareClass(HardwareMap hardwareMap){

        /*Chassis*/

        this.FR = hardwareMap.get(DcMotor.class , "FR");
        this.FL = hardwareMap.get(DcMotor.class , "FL");
        this.BR = hardwareMap.get(DcMotor.class , "BR");
        this.BL = hardwareMap.get(DcMotor.class , "BL");

        /*Subsystems*/

        this.ramp = hardwareMap.get(DcMotorEx.class, "Ramp");
        this.ramp2 = hardwareMap.get(DcMotorEx.class,"Ramp2");
        this.intakeMotor = hardwareMap.get(DcMotor.class, "IM");


        /*Servos*/
        this.angle = hardwareMap.get(Servo.class, "AG");
        this.selectTop = hardwareMap.get(Servo.class, "ST");
        this.selectBotR = hardwareMap.get(Servo.class, "SR");
        this.selectBotL = hardwareMap.get(Servo.class, "SL");
        this.turret1= hardwareMap.get(Servo.class,"TR1");
        this.turret2= hardwareMap.get(Servo.class,"TR2");

        /* Sensors */
        this.colorTop = hardwareMap.get(RevColorSensorV3.class, "CT");
        this.colorBotL = hardwareMap.get(RevColorSensorV3.class, "CL");
        this.colorBotR = hardwareMap.get(RevColorSensorV3.class, "CR");
        this.colorTop2 = hardwareMap.get(RevColorSensorV3.class, "CT2");
        this.colorBotL2 = hardwareMap.get(RevColorSensorV3.class, "CL2");
        this.colorBotR2 = hardwareMap.get(RevColorSensorV3.class, "CR2");
    }

    public static synchronized HardwareClass getInstance(HardwareMap hardwareMap){
        if(hardwareClass == null)
            hardwareClass = new HardwareClass(hardwareMap);
        return hardwareClass;
    }
}
