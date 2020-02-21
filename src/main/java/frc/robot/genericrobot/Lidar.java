package frc.robot.genericrobot;

import edu.wpi.first.hal.util.UncleanStatusException;
import edu.wpi.first.wpilibj.SerialPort;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.locks.ReentrantLock;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

class Lidar extends Thread {

    public Lidar() {
        start();
    }

    //Ex: "1234-5678"
    public static final Pattern lidarFormat = Pattern.compile(
        "(?<id>[0-9]{1,9})-(?<distance>[0-9]{1,9})"
    );

    private ReentrantLock dataLock = new ReentrantLock();
    public boolean isLocked() {
        return dataLock.isLocked();
    }

    private Map<Integer, Integer> lidarValues = new HashMap<>();

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
    public Double getDistanceInches(int lidarID) {
        Integer lidarReading = getDistance(lidarID);
        if (lidarReading == null) return  null;
        double lidarInches = 5+(15/396.0)*(lidarReading-165);
        return lidarInches;
    }

    private void writeDistance(int lidarID, int distance) {
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
                e.printStackTrace(System.out);
                System.out.println("We couldn't make the serial port. Will try again in a few seconds.");
                try {Thread.sleep(2000);}
                catch (Exception ignored) {}
            }
        }

        StringBuilder readText = new StringBuilder();
        System.out.println("Lidar started");

        while(true) {
            byte[] newByteArr = null;
            try {newByteArr = lidarSerialPort.read(1);}
            catch (UncleanStatusException e) {
                e.printStackTrace(System.out);
                try {Thread.sleep(1000);}
                catch (Exception ignored) {}
                continue;
            }
            if (newByteArr.length == 0) {
                System.out.println("Serial Port says we are at end of file. Nothing we can read.");
                System.out.println("Resetting.");
                lidarSerialPort.reset();
                continue;
            }
            byte newByte = newByteArr[0];

            if (newByte <     0) continue;
            if (newByte == '\n') continue;
            if (newByte == '\r') continue;
            if (newByte !=  ' ') {
                readText.append((char)newByte);
            } else {
                String lidarString = readText.toString();
                readText.delete(0,999);

                //Short circuit check: fail if no '-' in string
                if (!lidarString.contains("-")) continue;

                Matcher lidarBlob = lidarFormat.matcher(lidarString);
                if(!lidarBlob.matches()) continue;

                int id       = Integer.parseInt(lidarBlob.group(      "id"));
                int distance = Integer.parseInt(lidarBlob.group("distance"));

                System.out.printf(
                    "Found lidar %d with distance %d\n",
                    id,
                    distance
                );

                this.writeDistance(id, distance);

            }
        }
    }
}
