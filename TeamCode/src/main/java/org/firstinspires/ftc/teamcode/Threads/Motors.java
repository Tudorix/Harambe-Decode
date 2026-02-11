package org.firstinspires.ftc.teamcode.Threads;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClass;

public class Motors {

    private DcMotorEx ramp,ramp2;
    private DcMotor intakeMotor;

    private static Motors instance;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(39,0,0,1.2 ); //+- 20rpm close
    private Motors(HardwareClass hw) {
        ramp = hw.ramp;
        ramp2 = hw.ramp2;
        intakeMotor = hw.intakeMotor;

        ramp2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void intakeOn() {
        intakeMotor.setPower(1);
    }

    public void intakeOff() {
        intakeMotor.setPower(0);
    }

    public void intakeReverse(){
        intakeMotor.setPower(-1);
    }

    public void setRampVelocityC(int velocity) {
        ramp.setVelocity(velocity);
        ramp2.setVelocity(velocity);
    }
    public double getVelocity() {
        return (ramp.getVelocity()*60/28*0.625);
    }

    public double getRampError(double x){
        return getVelocity() - x;
    }

    public void rampStop() {
        ramp.setPower(0);
        ramp2.setPower(0);
    }

    public void setRampCoefs(){
        ramp.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d, MOTOR_VELO_PID.f));
        ramp2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d, MOTOR_VELO_PID.f));
        ramp2.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void setCoefsMan(double p, double i, double d, double f){
        ramp.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(p, i, d,f));
        ramp2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(p, i, d,f));
        ramp2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public static synchronized Motors getInstance(HardwareMap hw) {
        if (instance == null) {
            instance = new Motors(HardwareClass.getInstance(hw));
        }
        return instance;
    }
}
