package frc.robot.genericrobot;

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
        pixyCam.init();
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
            try {
                blockList.wait();
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
                    blocksList.clear();
                    System.gc();
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }


    public PixyCam.Block[] getPowerCellLocations() {
        PixyCam.Block[] blockList = this.blockList;
        blockList.notify();
        return blockList;
    }

    @Value public static class Block {
        public int signature, x, y, width, height, angle, index, age;
    }
}