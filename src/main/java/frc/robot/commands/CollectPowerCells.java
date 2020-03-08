package frc.robot.commands;

import frc.robot.genericrobot.GenericRobot;

public class CollectPowerCells{
    double collectorPower = 0.0;
    double escalatorPower = 0.0;
    long timeStart;
    long escalatorSpacing = 1000;
    public CollectPowerCells(){

    }

    public void begin(GenericRobot robot) {

    }

    public void run(GenericRobot robot) {
        collectorPower = 1.0;
        escalatorPower = 0.0;

        if (robot.getEscalatorSensorMedium()) {
            timeStart = System.currentTimeMillis();
            escalatorPower = 0.5;
        } else {
            if ((System.currentTimeMillis() >= timeStart + escalatorSpacing)) {
                escalatorPower = 0.0;
            }
            else {
                escalatorPower = 0.5;
            }
        }
        /*
        robot.escalatorUp(escalatorPower);
        if(robot.getEscalatorSensorMedium()){ //escalator moves only when medium is tripped
            escalatorPower = 0.5;
        } else {
            escalatorPower = 0.0;
        }*/

        robot.collectorIn(collectorPower);
        robot.escalatorUp(escalatorPower);
    }

    public void stop(GenericRobot robot) {
        robot.collectorIn(0);
        robot.escalatorUp(0);
    }
}
