package frc.robot.genericrobot;

import edu.wpi.first.wpilibj.SerialPort;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.locks.ReentrantLock;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class Lidar extends Thread {
    SerialPort lidarSerialPort = new SerialPort(
            9600,
            SerialPort.Port.kMXP,
            8,
            SerialPort.Parity.kNone,
            SerialPort.StopBits.kOne
    );

    Pattern lidarFormat = Pattern.compile("(?<id>[0-9]+)-(?<distance>[0-9]+)");

    Map<Integer, Integer> lidarValues = new HashMap<>();

    ReentrantLock dataLock = new ReentrantLock();

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
        StringBuffer readText = new StringBuffer();
        System.out.println("Lidar started");
        try {Thread.sleep(1000);}
        catch (Exception ignored) {}
        while(true) {
            byte newByte = lidarSerialPort.read(1)[0];
            //System.out.printf("Lidar read character '%x'\n", newByte);
            if (newByte == '\n') continue;
            if (newByte == '\r') continue;
            if (newByte != ' ') {
                readText.append((char)newByte);
            } else {
                String lidarString = readText.toString();

                System.out.printf("Lidar string is '%s'\n", lidarString);
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
