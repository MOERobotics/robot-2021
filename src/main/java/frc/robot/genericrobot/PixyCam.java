package frc.robot.genericrobot;

import frc.robot.Logger;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.links.SPILink;
import lombok.Value;
import java.util.ArrayList;

public class PixyCam implements Runnable {
    public boolean isRunning = false;
    private SPILink pixySPI = new SPILink();
    private Pixy2 pixyCam = Pixy2.createInstance(pixySPI);
    private Thread pixyThread;
    PixyCam.Block[] blockList = new PixyCam.Block[0];

    public void start() {
        switch (pixyCam.init()) {
            case Pixy2.PIXY_RESULT_OK:
                Logger.log("PIXY_INIT","Pixycam Connected");
                break;
            case Pixy2.PIXY_RESULT_ERROR: default:
                Logger.log("PIXY_INIT","Generic error connecting to Pixycam");
                break;
            case Pixy2.PIXY_RESULT_CHECKSUM_ERROR:
                Logger.log("PIXY_INIT","Checksum error connecting to Pixycam");
                break;
            case Pixy2.PIXY_RESULT_TIMEOUT:
                Logger.log("PIXY_INIT","Timeout error connecting to Pixycam");
                break;
            case Pixy2.PIXY_RESULT_BUTTON_OVERRIDE:
                Logger.log("PIXY_INIT","Button error connecting to Pixycam");
                break;
            case Pixy2.PIXY_RESULT_PROG_CHANGING:
                Logger.log("PIXY_INIT","Program error connecting to Pixycam");
                break;
        }
        pixyThread = new Thread(this);
        this.isRunning = true;
        pixyThread.start();
    }

    public void stop() {
        if (this.isRunning) {
            pixyThread.interrupt();
            pixyCam.close();
            this.isRunning = false;
        }
    }

    public void run() {
        while (this.isRunning) {
            synchronized (this) {
                try {
                    this.wait();
                    Logger.logTTL("PIXY_WAKE", "Looking for new data",300);
                    int ballsFound = pixyCam.getCCC().getBlocks(true);

                    switch (ballsFound) {
                        case Pixy2.PIXY_RESULT_ERROR:
                            Logger.logValueTTL("PIXY_READ", "Generic error connecting to Pixycam", ballsFound, 2000);
                            break;
                        case Pixy2.PIXY_RESULT_CHECKSUM_ERROR:
                            Logger.logValueTTL("PIXY_READ", "Checksum error connecting to Pixycam", ballsFound, 2000);
                            break;
                        case Pixy2.PIXY_RESULT_TIMEOUT:
                            Logger.logValueTTL("PIXY_READ", "Timeout error connecting to Pixycam", ballsFound, 2000);
                            break;
                        case Pixy2.PIXY_RESULT_BUTTON_OVERRIDE:
                            Logger.logValueTTL("PIXY_READ", "Button error connecting to Pixycam", ballsFound, 2000);
                            break;
                        case Pixy2.PIXY_RESULT_PROG_CHANGING:
                            Logger.logValueTTL("PIXY_READ", "Program error connecting to Pixycam", ballsFound, 2000);
                            break;
                        default:
                            if (ballsFound > 0) {
                                ArrayList<Pixy2CCC.Block> blocksList = pixyCam.getCCC().getBlockCache();
                                int i = 0;
                                PixyCam.Block[] ourBlockList = new PixyCam.Block[ballsFound];
                                for (Pixy2CCC.Block theirBlock : blocksList) {
                                    PixyCam.Block ourBlock = new Block(
                                        theirBlock.getSignature(),
                                        theirBlock.getX(),
                                        theirBlock.getY(),
                                        theirBlock.getWidth(),
                                        theirBlock.getHeight(),
                                        theirBlock.getAngle(),
                                        theirBlock.getIndex(),
                                        theirBlock.getAge()
                                    );
                                    ourBlockList[i++] = ourBlock;
                                }
                                this.blockList = ourBlockList;
                                blocksList.clear();
                                System.gc();
                            }
                            break;
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        }
    }


    public PixyCam.Block[] getPowerCellLocations() {
        PixyCam.Block[] blockList = this.blockList;
        synchronized (this) {this.notifyAll();}
        return blockList;
    }

    @Value public static class Block {
        public int signature, x, y, width, height, angle, index, age;

        public int getCenterX(){
            return x + (width/2);
        }
        public int getCenterY(){
            return y + (height/2);
        }
    }
}