package frc.robot.autonomous;

import frc.robot.genericrobot.GenericRobot;

public class DriveStraightFourFeet extends GenericAutonomous {
      double startingDistance;
      @Override public void autonomousPeriodic(GenericRobot robot) {
            switch (autonomousStep) {
                  case 0:
                        startingDistance = robot.getDistanceInchesLeft();
                        autonomousStep = 1;
                  case 1:
                        robot.driveForward(0.3);
                        double currentDistance = robot.getDistanceInchesLeft();
                        if (startingDistance - currentDistance > 12) {
                              autonomousStep = 2;
                        }
                        break;
                  case 2:
                        robot.driveForward(0);
                        break;
            }
      }
}
