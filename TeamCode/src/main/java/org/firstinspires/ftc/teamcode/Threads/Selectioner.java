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
    private Color resultTop, resultBotR, resultBotL;
    private ElapsedTime brat1 = new ElapsedTime(), brat2 = new ElapsedTime(), brat3 = new ElapsedTime();
    private ElapsedTime shootingTime = new ElapsedTime();
    private Thread thread = null;
    public boolean ballsfull = true;
    private Telemetry telemetry;
    private boolean isRunning = true, live = false;
    int brat1up, brat2up, brat3up, greenPos=-1, target = 1;
    public boolean shootingThreeBalls = false;

    public Selectioner(HardwareClass hardwareClass, Telemetry telemetry) {
        selectTop = hardwareClass.selectTop;   //1
        selectBotL = hardwareClass.selectBotL; //2
        selectBotR = hardwareClass.selectBotR; //3

        colorTop = hardwareClass.colorTop;     // 1
        colorBotL = hardwareClass.colorBotL;   // 2
        colorBotR = hardwareClass.colorBotR;   // 3

        colorTop2 = hardwareClass.colorTop2;     // 1
        colorBotR2 = hardwareClass.colorBotR2;   // 2
        colorBotL2 = hardwareClass.colorBotL2;   // 3

        this.telemetry = telemetry;
    }

    public void start() {       //starts color sensors
        isRunning = true;
        init();
        if(thread == null || !thread.isAlive()){
            thread = new Thread(() ->{
                    while(isRunning){
                        if(!shootingThreeBalls){
                            resultTop = Color.getColor2C(colorTop.red(),colorTop.green(),colorTop.blue(),colorTop.alpha(),colorTop2.red(),colorTop2.green(),colorTop2.blue(),colorTop2.alpha());
                            resultBotL = Color.getColor2C(colorBotL.red(),colorBotL.green(),colorBotL.blue(),colorBotL.alpha(),colorBotL2.red(),colorBotL2.green(),colorBotL2.blue(),colorBotL2.alpha());
                            resultBotR= Color.getColor2C(colorBotR.red(),colorBotR.green(),colorBotR.blue(),colorBotR.alpha(),colorBotR2.red(),colorBotR2.green(),colorBotR2.blue(),colorBotR2.alpha());
                        }
                        if(resultTop!= Color.INVALID && resultBotL != Color.INVALID && resultBotR != Color.INVALID)
                            ballsfull = true;
                        else
                            ballsfull = false;
                        checkTimer();
                        sleep(30);
                    }
            });
        }
        thread.start();
    }
    void checkTimer(){  //checks servos
        if(brat1.milliseconds()>HardwareClass.bratDelay && brat1up == 1) {
            selectTop.setPosition(HardwareClass.selectTopREST);
            brat1up=2;
            live = false;
        }
        if(brat2.milliseconds()>HardwareClass.bratDelay && brat2up == 1) {
            selectBotL.setPosition(HardwareClass.selectBotLREST);
            brat2up=2;
            live = false;
        }
        if(brat3.milliseconds()>HardwareClass.bratDelay && brat3up == 1) {
            selectBotR.setPosition(HardwareClass.selectBotRREST);
            brat3up=2;
            live = false;
        }
        if(brat1up == 2 && brat2up == 2 && brat3up == 2){
            selectTop.setPosition(HardwareClass.selectTopLOW);
            selectBotL.setPosition(HardwareClass.selectBotLLOW);
            selectBotR.setPosition(HardwareClass.selectBotRLOW);
            brat1up=0;
            brat2up=0;
            brat3up=0;
        }
    }

    public void resetBrat(){  //resets servos
            selectTop.setPosition(HardwareClass.selectTopLOW);
            selectBotL.setPosition(HardwareClass.selectBotLLOW);
            selectBotR.setPosition(HardwareClass.selectBotRLOW);
            brat1up=0;
            brat2up=0;
            brat3up=0;
    }

    public void shoot3Balls(){  //shoots 3 balls using the obelisk colors
        shootingThreeBalls = true;
        shootingTime.reset();
        printesaDinDubai(greenPos);
        shootingThreeBalls = false;
    }

    public void shootingBalls(){
        if(!live){
            if(greenPos==1 && target == 1){
                shootGreen();
            } else {
                shootPurple();
            }
            if(greenPos==2 && target == 2){
                shootGreen();
            } else {
                shootPurple();
            }
            if(greenPos==3 && target == 3){
                shootGreen();
            } else {
                shootPurple();
            }
            target++;
        }
    }

    public void printesaDinDubai(int pos){  //deprecated way of shooting
        if(pos<0) {
            unloadBalls();
            return;
        }
        if(pos==1) shootGreen(); else shootPurple();
        sleep(HardwareClass.bratDelay + 300);
        if(pos==2) shootGreen(); else shootPurple();
        sleep(HardwareClass.bratDelay + 300);
        if(pos==3) shootGreen(); else shootPurple();// doublecheck pentru surplus!!!!
        sleep(HardwareClass.bratDelay + 300);
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



    public void rightServoUp(){
        brat3.reset();
        brat3up = 1;
        selectBotR.setPosition(HardwareClass.selectBotRHIGH);
    }

    public void leftServoUp(){
        brat2.reset();
        brat2up = 1;
        selectBotL.setPosition(HardwareClass.selectBotLHIGH);
    }

    public void topServoUp() {
        brat1.reset();
        brat1up = 1;
        selectTop.setPosition(HardwareClass.selectTopHIGH);
    }
    public void shootPurple() {
        if(resultTop == Color.PURPLE) {
            resultTop = Color.INVALID;
            topServoUp();
        } else if(resultBotL == Color.PURPLE) {
            resultBotL = Color.INVALID;
            leftServoUp();
        } else if(resultBotR == Color.PURPLE) {
            resultBotR = Color.INVALID;
            rightServoUp();
        }
    }

    public void shootGreen() {
        if(resultTop == Color.GREEN) {
            resultTop = Color.INVALID;
            topServoUp();
        } else if(resultBotL == Color.GREEN) {
            resultBotL = Color.INVALID;
            leftServoUp();
        } else if(resultBotR == Color.GREEN) {
            resultBotR = Color.INVALID;
            rightServoUp();
        }
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

    public void init(){
        selectBotR.setDirection(Servo.Direction.FORWARD);
        selectBotL.setDirection(Servo.Direction.FORWARD);
        selectBotR.setPosition(HardwareClass.selectBotRLOW);
        selectBotL.setPosition(HardwareClass.selectBotLLOW);
        selectTop.setPosition(HardwareClass.selectTopLOW);
    }


    public void setObeliskColors(int pos) {  //must be run on start of opMode, runs unload if not set
        greenPos = pos;
    }

    public void stop() {
        isRunning = false;
        if (thread != null ) {
            thread.interrupt();
        }
    }

    public void pause(){
        isRunning = false;
    }
    public void aladam(){
        isRunning = true;
    }

    public void sleep(int sec){
        try {
            Thread.sleep(sec);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public static synchronized Selectioner getInstance(HardwareClass hardwareClass, Telemetry telemetry) {
        if(selectionerInstance == null) {
            selectionerInstance = new Selectioner(hardwareClass, telemetry);
        }

        return selectionerInstance;
    }
}