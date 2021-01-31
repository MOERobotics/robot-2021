package frc.robot.autonomous;

import frc.robot.genericrobot.GenericRobot;

import java.io.*;

public class RepeatRecordedPath extends GenericAutonomous {

    String FileName;

    public RepeatRecordedPath (String FileName){
        this.FileName = FileName;
    }

    BufferedReader input;

    {
        try {
            input = new BufferedReader(new FileReader(FileName));
        } catch (Exception e) {
            // I don't know how to display the exception.
        }
    }

    String leftPowerString;
    String rightPowerString;
    double leftPower = 0;
    double rightPower = 0;
    @Override
    public void autonomousInit(GenericRobot robot) {
        {
            try {
                input = new BufferedReader(new FileReader(FileName));
            } catch (Exception e) {
                // I don't know how to display the exception.
            }
        }
    }
    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        try {
            leftPowerString = input.readLine();
            rightPowerString = input.readLine();
        } catch (IOException e) {
            // I don't know how to display the exception.
        }
        if (leftPowerString != null && rightPowerString != null) {
            leftPower = Double.parseDouble(leftPowerString);
            rightPower = Double.parseDouble(rightPowerString);
            robot.setMotorPowerPercentage(leftPower, rightPower);
        } else {
            try {
                input.close();
            } catch (IOException e) {
                // I don't know how to display the exception.
            }
        }
    }
}
