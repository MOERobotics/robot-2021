package frc.robot.autonomous;

import frc.robot.genericrobot.GenericRobot;

import java.io.*;

public class RepeatRecordedPath extends GenericAutonomous{

    BufferedReader input;
    {
        try {
            input = new BufferedReader(new FileReader("Autonomous1.txt"));
        }
        catch (FileNotFoundException e) {
            // I don't know how to display the exception.
        }
    }

    String leftPowerString;
    String rightPowerString;
    double leftPower = 0;
    double rightPower = 0;

    @Override public void autonomousPeriodic(GenericRobot robot) {
        switch (autonomousStep) {
            case 0:
                try {
                    leftPowerString = input.readLine();
                    rightPowerString = input.readLine();
                }
                catch (IOException e) {
                    // I don't know how to display the exception.
                }
                if(leftPowerString != null && rightPowerString != null){
                    leftPower = Double.parseDouble(leftPowerString);
                    rightPower = Double.parseDouble(rightPowerString);
                    robot.setMotorPowerPercentage(leftPower, rightPower);
                }
                else{
                    autonomousStep = 1;
                }
                break;

            case 1:
                try {
                    input.close();
                }
                catch (IOException e) {
                    // I don't know how to display the exception.
                }
                break;
        }
    }
}
