package frc.robot.commands;

import frc.robot.genericrobot.GenericRobot;

public class SmartClimb{

    boolean engagedPort = false;
    boolean engagedStarboard = false;
    double climbPowerPort;
    double climbPowerStarboard;
    double rollTolerance=3.0;
    double currentTolerance = 5.0;

    public SmartClimb(){

    }

    public void begin(GenericRobot robot) {
        engagedPort = false;
        engagedStarboard = false;
    }

    public void step(GenericRobot robot) {
        climbPowerPort = 0.4;
        climbPowerStarboard = 0.4;

        if (Math.abs(robot.getClimberVerticalPortCurrent()) > currentTolerance) { engagedPort = true;}
        if (Math.abs(robot.getClimberVerticalStarboardCurrent()) > currentTolerance) { engagedStarboard = true;}

        if (engagedPort)
        {
            climbPowerPort = 0.0;
        }

        if (engagedStarboard)
        {
            climbPowerPort = 0.0;
        }

        if (engagedPort && engagedStarboard)
        {
            climbPowerPort = 0.8;
            climbPowerStarboard = 0.8;

            if (robot.getRoll()> rollTolerance)
            {
                climbPowerStarboard += 0.2;
            }
            if (robot.getRoll() < -rollTolerance)
            {
                climbPowerPort += 0.2;
            }
        }

        robot.setClimbVerticalPortPower(-climbPowerPort);
        robot.setClimbVerticalPortPower(-climbPowerStarboard);
    }


}
