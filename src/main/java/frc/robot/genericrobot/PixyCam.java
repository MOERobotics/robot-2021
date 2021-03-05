package frc.robot.genericrobot;

import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2Line;
import io.github.pseudoresonance.pixy2api.links.SPILink;

import lombok.Value;

import java.util.ArrayList;
import java.util.Arrays;

public class PixyCam implements Runnable {
    public volatile boolean isRunning = false;
    private SPILink pixySPI = new SPILink();
    private Pixy2 pixyCam = Pixy2.createInstance(pixySPI);
    private Thread pixyThread;

    PixyCam.Block[] blockList = null;

    public void init() {
        //Init pixycam
        pixyCam.init();
    }

    public void start() {
        pixyThread = new Thread(this);
        this.isRunning = true;
        pixyThread.start();
    }

    public void stop() {
        if (this.isRunning) {
            this.isRunning = false;
            pixyThread.interrupt();
        }
    }

    public void run() {
        while (this.isRunning) {
            try {
                int ballsFound = pixyCam.getCCC().getBlocks(true);
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
                        ourBlockList[i] = ourBlock;
                    }
                    this.blockList = ourBlockList;
                }
                pixyCam.getLine().getAllFeatures();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }


    public PixyCam.Block[] getPowerCellLocations() {

        return null;
    }

    @Value public static class Block {
        public int signature, x, y, width, height, angle, index, age;
    }
}