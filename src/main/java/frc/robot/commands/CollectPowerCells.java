package frc.robot.commands;

import frc.robot.genericrobot.GenericRobot;

public class CollectPowerCells{
    double collectorPower = 0.0;
    double escalatorPower = 0.0;
    public CollectPowerCells(){


    }

    public void begin(GenericRobot robot) {

    }

    public void run(GenericRobot robot) {
        collectorPower = 1.0;

        if(robot.getElevatorSensorMedium()){ //escalator moves only when medium is tripped
            escalatorPower = 0.5;
        } else {
            escalatorPower = 0.0;
        }

        robot.collectorIn(collectorPower);
        robot.escalatorUp(escalatorPower);

    }


}
