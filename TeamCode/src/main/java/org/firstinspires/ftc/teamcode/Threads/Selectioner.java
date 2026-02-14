package org.firstinspires.ftc.teamcode.Threads;

import android.health.connect.datatypes.SleepSessionRecord;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.exceptions.ImuNotFoundException;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareClass;
import org.firstinspires.ftc.teamcode.Utils.Color;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.GregorianCalendar;
import java.util.Iterator;
import java.util.List;

public class Selectioner{
    private static Selectioner selectionerInstance = null;
    private Servo selectTop, selectBotR, selectBotL;
    private RevColorSensorV3 colorTop, colorBotR, colorBotL,colorTop2, colorBotR2, colorBotL2;
    private volatile Color resultTop, resultBotR, resultBotL;
    public boolean ballsfull = true;
    private Telemetry telemetry;
    int brat1up, brat2up, brat3up;

    public Selectioner(HardwareClass hardwareClass, Telemetry telemetry) {
        selectTop = hardwareClass.selectTop;   // 1
        selectBotL = hardwareClass.selectBotL; // 2
        selectBotR = hardwareClass.selectBotR; // 3

        colorTop = hardwareClass.colorTop;       // 1
        colorBotL = hardwareClass.colorBotL;     // 2
        colorBotR = hardwareClass.colorBotR;     // 3

        colorTop2 = hardwareClass.colorTop2;     // 1
        colorBotR2 = hardwareClass.colorBotR2;   // 2
        colorBotL2 = hardwareClass.colorBotL2;   // 3

        this.telemetry = telemetry;
    }


    public void printesaDinDubai(int pos){  //deprecated way of shooting
        if(pos<0) {
            unloadBalls();
            return;
        }
    }

    public void unloadBalls(){  //
        rightServoUp();
        sleep(355);
        leftServoUp();
        sleep(355);
        topServoUp();
    }

    public void unloadBallsSlow(){  //
        rightServoUp();
        sleep(600);
        leftServoUp();
        sleep(600);
        topServoUp();
        sleep(200);
    }

    public void unloadBallsQuick(){  //
        sleep(100);
        rightServoUp();
        sleep(100);
        leftServoUp();
        sleep(140);
        topServoUp();
    }

    public void unloadBallsQuick_Short(){  //
        rightServoUp();
        sleep(100);
        leftServoUp();
        sleep(140);
        topServoUp();
    }

    public int getOrder(){
        return 0;
    }

    public void topServoUp() {
        selectTop.setPosition(HardwareClass.selectTopHIGH);
        brat1up = 1;
        sleep(HardwareClass.bratDelay);
        selectTop.setPosition(HardwareClass.selectTopLOW);
    }

    public void resetServos(){
        selectTop.setPosition(HardwareClass.selectTopLOW);
        selectBotL.setPosition(HardwareClass.selectBotLLOW);
        selectBotR.setPosition(HardwareClass.selectBotRLOW);
    }

    public void leftServoUp(){
        selectBotL.setPosition(HardwareClass.selectBotLHIGH);
        brat2up = 1;
        sleep(HardwareClass.bratDelay);
        selectBotL.setPosition(HardwareClass.selectBotLLOW);
    }

    public void rightServoUp(){
        selectBotR.setPosition(HardwareClass.selectBotRHIGH);
        brat3up = 1;
        sleep(HardwareClass.bratDelay);
        selectBotR.setPosition(HardwareClass.selectBotRLOW);
    }

    public int getPurplePos() {
        if(resultTop == Color.PURPLE) {
            return 1;
        } else if(resultBotL == Color.PURPLE) {
            return 2;
        } else if(resultBotR == Color.PURPLE) {
            return 3;
        }
        return -1;
    }

    public int getGreenPos() {
        if(resultTop == Color.GREEN) {
            return 1;
        } else if(resultBotL == Color.GREEN) {
            return 2;
        } else if(resultBotR == Color.GREEN) {
            return 3;
        }
        return -1;
    }

    public void telemetry() {
        telemetry.addData("Top status:", brat1up);
        telemetry.addData("Left status:", brat2up);
        telemetry.addData("Right status:", brat3up);
        telemetry.addData("Top Color", resultTop);
        telemetry.addData("Right Color", resultBotR);
        telemetry.addData("Left Color:", resultBotL);
        telemetry.addData("Top R", colorTop.red());
        telemetry.addData("Top G", colorTop.green());
        telemetry.addData("Top B", colorTop.blue());
        telemetry.addData("Top A", colorTop.alpha());
        telemetry.addData("Left R", colorBotL.red());
        telemetry.addData("Left G", colorBotL.green());
        telemetry.addData("Left B", colorBotL.blue());
        telemetry.addData("Left A", colorBotL.alpha());
        telemetry.addData("Right R", colorBotR.red());
        telemetry.addData("Right G", colorBotR.green());
        telemetry.addData("Right B", colorBotR.blue());
        telemetry.addData("Right A", colorBotR.alpha());
    }

    public void checkColors(){  //external way of checking, deprecated
        resultTop = Color.getColor2C(colorTop.red(),colorTop.green(),colorTop.blue(),colorTop.alpha(),colorTop2.red(),colorTop2.green(),colorTop2.blue(),colorTop2.alpha());
        resultBotL = Color.getColor2C(colorBotL.red(),colorBotL.green(),colorBotL.blue(),colorBotL.alpha(),colorBotL2.red(),colorBotL2.green(),colorBotL2.blue(),colorBotL2.alpha());
        resultBotR= Color.getColor2C(colorBotR.red(),colorBotR.green(),colorBotR.blue(),colorBotR.alpha(),colorBotR2.red(),colorBotR2.green(),colorBotR2.blue(),colorBotR2.alpha());
    }


    public void sleep(int sec){
        try {
            Thread.sleep(sec);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public static synchronized Selectioner getInstance(HardwareClass hardwareClass, Telemetry telemetry) {
        if(selectionerInstance == null) {
            selectionerInstance = new Selectioner(hardwareClass, telemetry);
        }

        return selectionerInstance;
    }
}