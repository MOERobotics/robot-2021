package frc.robot.commands;

import frc.robot.genericrobot.GenericRobot;

public class CollectPowerCells extends GenericCommand{
    double collectorPower = 0.0;
    double escalatorPower = 0.0;
    public CollectPowerCells(){


    }
    @Override
    public void begin(GenericRobot robot) {

    }

    @Override
    public void step(GenericRobot robot) {
        collectorPower = 1.0;


        if(robot.getElevatorSensorMedium()){
            escalatorPower = 0.5;
        } else {
            escalatorPower = 0.0;
        }

        robot.collectorIn(collectorPower);
        robot.escalatorUp(escalatorPower);

    }


}
