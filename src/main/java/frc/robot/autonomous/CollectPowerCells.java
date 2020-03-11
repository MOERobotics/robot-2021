package frc.robot.autonomous;

import frc.robot.genericrobot.GenericRobot;

public class CollectPowerCells {
    double collectorPower = 0.0;
    double escalatorPower = 0.0;
    boolean waitingForMediumHigh = false;
    boolean waitingForChange = false;

    public CollectPowerCells(){

    }

    public void begin(GenericRobot robot) {
        waitingForMediumHigh = false;
        waitingForChange = false;
    }

    public void run(GenericRobot robot) {
        escalatorPower = 0.0;
        collectorPower = 0.75;

        //two power cells
        if (robot.getEscalatorSensorLow() && robot.getEscalatorSensorMedium() && !robot.getEscalatorSensorMediumHigh()){
            waitingForMediumHigh = true;
        }

        //three and four power cells
        if (robot.getEscalatorSensorLow() && robot.getEscalatorSensorMedium() && robot.getEscalatorSensorMediumHigh() && !robot.getEscalatorSensorHigh()){
            waitingForChange = true;
            waitingForMediumHigh = false;
        }
        if (waitingForChange && !robot.getEscalatorSensorMediumHigh()){
            waitingForChange = false;
            waitingForMediumHigh = true;
        }
        if (waitingForMediumHigh || waitingForChange){
            escalatorPower = 0.7;
        }
        if (waitingForMediumHigh && robot.getEscalatorSensorMediumHigh()){
            waitingForMediumHigh = false;
        }

        if (robot.getEscalatorSensorLow() && robot.getEscalatorSensorMedium() && robot.getEscalatorSensorMediumHigh() && robot.getEscalatorSensorHigh()){
            collectorPower = 0.0;
            escalatorPower = 0.0;
        }

        robot.collectorIn(collectorPower);
        robot.escalatorUp(escalatorPower);

    }

    public void stop(GenericRobot robot) {
        robot.collectorIn(0);
        robot.escalatorUp(0);
    }
}
