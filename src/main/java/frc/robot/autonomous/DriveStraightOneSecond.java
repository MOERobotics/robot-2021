package frc.robot.autonomous;

import frc.robot.genericrobot.GenericRobot;

public class DriveStraightOneSecond extends GenericAutonomous {

      long startingTime;

      @Override public void autonomousPeriodic(GenericRobot robot) {
            switch (autonomousStep) {
                  case 0:
                        startingTime = System.currentTimeMillis();
                        autonomousStep = 1;
                  case 1:
                        robot.driveForward(0.3);
                        long currentTime = System.currentTimeMillis();
                        if (currentTime - startingTime > 1000) {
                              autonomousStep = 2;
                        }
                        break;
                  case 2:
                        robot.driveForward(0);
                        break;
            }
      }
}

/*

      R: -3296
      L: 3206

      R: 27.5"
      L: 29.0"



 */
