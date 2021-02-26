package frc.robot.genericrobot;

import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2Line;
import io.github.pseudoresonance.pixy2api.links.SPILink;

import java.util.ArrayList;

public class PixyCam implements Runnable{
    public volatile boolean isRunning = false;
    private SPILink pixySPI = new SPILink();
    private Pixy2 pixyCam = Pixy2.createInstance(pixySPI);
    private Thread pixyThread;

    public Pixy2Line.Vector[] vec;

    private Pixy2Line.Vector[] sillyNullVector = new Pixy2Line.Vector[0];

    public void init(){
        //Init pixycam
        pixyCam.init();
    }
    public void start(){
        pixyThread = new Thread(this);
        this.isRunning = true;
        pixyThread.start();
    }
    public void stop(){
        if(this.isRunning){
            this.isRunning = false;
            pixyThread.interrupt();
        }
    }
    public void run(){
        while(this.isRunning){
            try{
                int ballsFound = pixyCam.getCCC().getBlocks();
                if (ballsFound > 0){
                    ArrayList<Pixy2CCC.Block> blocksList = pixyCam.getCCC().getBlockCache();
                    for (Pixy2CCC.Block block: blocksList){

                    }
                }
                pixyCam.getLine().getAllFeatures();
                Pixy2Line.Vector[] tmp = pixyCam.getLine().getVectorCache();
                if (tmp == null) tmp = sillyNullVector;
                vec = tmp;
                Thread.sleep(100);
            }catch(Exception e){
                e.printStackTrace();
            }
        }
    }
    public Pixy2Line.Vector[] getLastVector(){
        synchronized (this) {
            return vec;
        }
    }

    public Object getPowerCellLocations(){

        return null;
    }

    @Override
    public String toString() {
        StringBuilder pixyOutput = new StringBuilder();

        pixyOutput.append("[");
        boolean first = true;
        //copy ptr
        Pixy2Line.Vector[] vectors = this.vec;
        for (Pixy2Line.Vector vec : vectors) {
            if (!first) pixyOutput.append(",");
            pixyOutput.append(String.format(
                    "{"+
                            "X0: %d, "+
                            "Y0: %d, "+
                            "X1: %d, "+
                            "Y1: %d"+
                            "}",
                    vec.getX0(),
                    vec.getY0(),
                    vec.getX1(),
                    vec.getY1()
            ));
            first=false;
        }
        pixyOutput.append("]");

        return pixyOutput.toString();

    }
}
