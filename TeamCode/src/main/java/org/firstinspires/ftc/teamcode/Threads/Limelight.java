package org.firstinspires.ftc.teamcode.Threads;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareClass;


public class Limelight {
    Thread thread = null;
    private static Limelight single_instance = null;
    private boolean running = false;
    public double distance;
    public int pipeline = -1;
    private Limelight3A limelight;
    double Tx, Ty;
    public Limelight(HardwareClass hardwareClass, Telemetry telemetry , HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, "Ethernet Device");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void start(){
        running = true;
        if(thread == null || !thread.isAlive()){
            thread = new Thread(() ->{
                while(true){
                    if(running && pipeline <=0){
                        int tryPipe = 1;
                        for (int attempts = 0; attempts < 6; attempts++) {
                            LLResult result = limelight.getLatestResult();
                            if (result.isValid()) {
                                pipeline = tryPipe;
                            }

                            tryPipe = (tryPipe == 3) ? 1 : tryPipe + 1;
                            setPipeline(tryPipe);

                            try {
                                Thread.sleep(200);
                            } catch (InterruptedException e) {
                                Thread.currentThread().interrupt();
                                break;
                            }
                        }
                    }
                }
            });
        }
        thread.start();
    }

    public double getXPos(){
        LLResult result = limelight.getLatestResult();
        Tx = result.getTx();
        return Tx;
    }

    public boolean checkResults(){
        LLResult result = limelight.getLatestResult();
        return result.isValid();
    }
    public double getYPos() {
        LLResult result = limelight.getLatestResult();
        Ty = result.getTy();
        return Ty;
    }

    public double getArea(){
        LLResult result = limelight.getLatestResult();
        return result.getTa();
    }

    public int checkApriltagResults() {
        return pipeline;
    }

    public double getDistanceOD(double x, double y, int color){ //Formula automata pentru cele doua cosuri
        if(color == 0){//red
            distance = Math.sqrt(Math.pow(HardwareClass.redX-x,2)+Math.pow(HardwareClass.redY-y,2));
        }
        else
            distance = Math.sqrt(Math.pow(HardwareClass.blueX-x,2)+Math.pow(HardwareClass.blueY-y,2));

        return distance * 2.54;
    }


    public void setPipeline(int pipe){
        limelight.pipelineSwitch(pipe);
    }

    public void stop(){
        running = false;
    }
    public void setup(){
        running = true;
    }
    public boolean getStatus() {
        return running;
    }
    public static synchronized Limelight getInstance(HardwareMap hardwareMap , Telemetry telemetry ){
        if(single_instance == null){
            single_instance = new Limelight(HardwareClass.getInstance(hardwareMap), telemetry, hardwareMap);
        }
        return single_instance;
    }
}
