package frc.robot.genericrobot;

import edu.wpi.first.wpilibj.SerialPort;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.locks.ReentrantLock;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class Lidar extends Thread {

    static final Pattern lidarFormat = Pattern.compile("(?<id>[0-9]{1,9})-(?<distance>[0-9]{1,9})");

    Map<Integer, Integer> lidarValues = new HashMap<>();

    ReentrantLock dataLock = new ReentrantLock();

    public boolean isLocked() {
        return dataLock.isLocked();
    }

    public Integer getDistance(int lidarID) {
        Integer distance = null;
        try {
            dataLock.lock();
            if(lidarValues.containsKey(lidarID)) {
                distance = lidarValues.get(lidarID);
            }
        } finally {
            dataLock.unlock();
        }
        return distance;
    }

    public void writeDistance(int lidarID, int distance) {
        try {
            dataLock.lock();
            lidarValues.put(lidarID,distance);
        } finally {
            dataLock.unlock();
        }
    }

    @Override public void run() {
        this.setName("Lidar Thread");

        SerialPort lidarSerialPort = null;
        while (lidarSerialPort == null) {
            try {
                lidarSerialPort = new SerialPort(
                        9600,
                        SerialPort.Port.kMXP,
                        8,
                        SerialPort.Parity.kNone,
                        SerialPort.StopBits.kOne
                );
            } catch (Exception e) {
                System.out.println(e);
                System.out.println("We couldn't make the serial port. Will try again in a few seconds.");
                try {
                    Thread.sleep(2000);
                } catch (Exception ignored) {}
            }
        }
        //Throws exception if it cannot open the serial port -- Maybe try again in a few seconds?

        StringBuffer readText = new StringBuffer();
        try {Thread.sleep(1000);}
        catch (Exception ignored) {}
        System.out.println("Lidar started");
        while(true) {
            byte[] newByteArr = lidarSerialPort.read(1);
            if (newByteArr.length == 0) {
                System.out.println("Serial Port says we are at end of file. Nothing we can read.");
                return;
            }
            byte newByte = newByteArr[0];
            //Can throw if lidarSerialPort isn't feeling good today --Maybe we can try to reconnect to the serial bus
            //Can throw if lidarSerialPort is at EOF (zero length array) -- I don't think we can recover from


            if (newByte < 0) continue;
            if (newByte == '\n') continue;
            if (newByte == '\r') continue;
            //System.out.printf("Lidar read character '%x'\n", newByte);
            if (newByte != ' ') {
                readText.append((char)newByte);
            } else {
                String lidarString = readText.toString();
                readText.delete(0,999);

                if (!lidarString.contains("-")) {
                    continue;
                }

                Matcher lidarBlob = lidarFormat.matcher(lidarString);

                if(!lidarBlob.matches()) {
                    continue;
                }

                int id = Integer.parseInt(lidarBlob.group("id"));
                int distance = Integer.parseInt(lidarBlob.group("distance"));

                System.out.printf("Found lidar %d with distance %d\n", id, distance);
                writeDistance(id, distance);

            }
        }
    }
}
