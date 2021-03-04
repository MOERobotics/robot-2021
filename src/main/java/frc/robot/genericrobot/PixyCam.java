package frc.robot.genericrobot;

import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2Line;
import io.github.pseudoresonance.pixy2api.links.SPILink;

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
                int ballsFound = pixyCam.getCCC().getBlocks();
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

                Thread.sleep(100);
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }


    public PixyCam.Block[] getPowerCellLocations() {

        return null;
    }

    public static class Block {

        public final int signature, x, y, width, height, angle, index, age;

        /**
         * Constructs signature block instance
         *
         * @param signature Block signature
         * @param x         X value
         * @param y         Y value
         * @param width     Block width
         * @param height    Block height
         * @param angle     Angle from camera
         * @param index     Block index
         * @param age       Block age
         */

        private Block(int signature, int x, int y, int width, int height, int angle, int index, int age) {
            this.signature = signature;
            this.x = x;
            this.y = y;
            this.width = width;
            this.height = height;
            this.angle = angle;
            this.index = index;
            this.age = age;
        }

        public void print() {
            System.out.println(toString());
        }

        public String toString() {
            String out = "";
            out = "sig: " + signature + " x: " + x + " y: " + y + " width: " + width + " height: " + height
                    + " index: " + index + " age: " + age;
            return out;
        }


        public int getSignature() {
            return signature;
        }

        public int getX() {
            return x;
        }

        public int getY() {
            return y;
        }

        public int getWidth() {
            return width;
        }

        public int getHeight() {
            return height;
        }

        public int getAngle() {
            return angle;
        }

        public int getIndex() {
            return index;
        }

        public int getAge() {
            return age;
        }

    }
}