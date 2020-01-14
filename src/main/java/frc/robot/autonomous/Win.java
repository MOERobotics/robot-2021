package frc.robot.autonomous;

import frc.robot.genericrobot.GenericRobot;

public class Win extends GenericAutonomous {

      double startingYaw      = 0.0;
      double startingDistance = 0.0;

      @Override public void autonomousPeriodic(GenericRobot robot) {
            double currentYaw = 0;
            double currentDistance = 0;
            switch (autonomousStep) {
                  case 0:
                        robot.resetTicksLeft();
                        robot.resetTicksRight();
                        robot.setShooterPowerPercentage(1);
                        if (true) autonomousStep = 2;
                        break;
                  case 1:
                  case 2:
                        startingYaw = robot.getYaw();
                        autonomousStep = 3;
                  case 3:
                        robot.driveRightInPlace(0.5);
                        currentYaw = robot.getYaw();
                        if (currentYaw - startingYaw > 90) {
                              robot.driveForward(0);
                              autonomousStep = 4;
                        } else break;
                  case 4:
                        startingDistance = robot.getDistanceInchesLeft();
                        autonomousStep = 5;
                  case 5:
                        robot.driveForward(0.5);
                        currentDistance = robot.getDistanceInchesLeft();
                        if (currentDistance - startingDistance > 66.91) {
                              robot.driveForward(0);
                              autonomousStep = 6;
                        } else break;
                  case 6:
                        startingYaw = robot.getYaw();
                        autonomousStep = 7;
                  case 7:
                        robot.driveLeftInPlace(0.5);
                        currentYaw = robot.getYaw();
                        if (currentYaw - startingYaw < -90) {
                              robot.driveForward(0);
                              autonomousStep = 8;
                        } else break;
                  case 8:
                        startingDistance = robot.getDistanceInchesLeft();
                        autonomousStep = 9;
                  case 9:
                        robot.driveReverse(0.5);
                        currentDistance = robot.getDistanceInchesLeft();
                        if (currentDistance - startingDistance > -195) {
                              robot.driveForward(0);
                              autonomousStep = 10;
                        } else break;
                  case 10:
                        robot.driveForward(0);
                        //                               ¯\_(ツ)_/¯
                        break;
            }
      }
}
