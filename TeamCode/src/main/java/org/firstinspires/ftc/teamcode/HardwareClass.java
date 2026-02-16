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
    public static double redX = 132, redY = 132;
    public static double redScoreX = 135, redScoreY = 135;//65   62
    public static double blueX = 8, blueY = 8;
    public static double blueScoreX = 135, blueScoreY = 9;

    public static int bratDelay = 180,bratDownDelay = 150; // milisecunde

    public static  double selectTopLOW = 0.86, selectBotLLOW = 0.89,selectBotRLOW =0.93 ;//0.14

    public static  double selectTopREST = 0.68, selectBotLREST = 0.72,selectBotRREST =0.77 ;//0.35 //Intermediate to make sure it reads color correctly         calibrate!

    public static  double selectTopHIGH = 0.4, selectBotLHIGH = 0.45,selectBotRHIGH = 0.50;//0.57

    public double hood_down = 1 , hood_up = 0.7;



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
        this.turret = hardwareMap.get(DcMotorEx.class , "TR");


        /*Servos*/
        this.angle = hardwareMap.get(Servo.class, "AG");
        this.selectTop = hardwareMap.get(Servo.class, "ST");
        this.selectBotR = hardwareMap.get(Servo.class, "SR");
        this.selectBotL = hardwareMap.get(Servo.class, "SL");

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
