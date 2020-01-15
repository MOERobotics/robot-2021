package frc.robot.autonomous;

import frc.robot.PIDModule;
import frc.robot.genericrobot.GenericRobot;

public class Win extends GenericAutonomous {

      double startingYaw      = 0.0;
      double startingDistance = 0.0;
      PIDModule PIDSteering = new PIDModule(0.06, 1.0e-3, 1.0e-4);
      double correction;

      @Override public void autonomousPeriodic(GenericRobot robot) {
            double currentYaw = 0;
            double currentDistance = 0;
            switch (autonomousStep) {
                  case 0:
                        //robot.setShooterPowerPercentage(1);
                        if (true) autonomousStep = 2;
                        break;
                  case 1:
                  case 2:
                        startingYaw = robot.getYaw();
                        autonomousStep = 3;
                  case 3:
                        robot.driveLeftInPlace(0.2);
                        currentYaw = robot.getYaw();
                        if (currentYaw - startingYaw < -90) {
                              robot.driveForward(0);
                              currentYaw = robot.getYaw();
                              autonomousStep = 4;
                        } else break;
                  case 4:
                        startingDistance = robot.getDistanceInchesLeft();
                        PIDSteering.resetError();
                        autonomousStep = 5;
                  case 5:
                        PIDSteering.setHeading(robot.getYaw()-currentYaw);
                        correction = PIDSteering.getCorrection();
                        robot.setMotorPowerPercentage(0.2*(1+correction), 0.2*(1-correction));
                        currentDistance = robot.getDistanceInchesLeft();
                        if (currentDistance - startingDistance > 66.91) {
                              robot.driveForward(0);
                              autonomousStep = 6;
                        } else break;
                  case 6:
                        startingYaw = robot.getYaw();
                        autonomousStep = 7;
                  case 7:
                        robot.driveRightInPlace(0.2);
                        currentYaw = robot.getYaw();
                        if (currentYaw - startingYaw > 90) {
                              robot.driveForward(0);
                              autonomousStep = 8;
                        } else break;
                  case 8:
                        startingDistance = robot.getDistanceInchesLeft();
                        autonomousStep = 9;
                  case 9:
                        robot.driveForward(0.2);
                        currentDistance = robot.getDistanceInchesLeft();
                        if (currentDistance - startingDistance > 195) {
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



/*

      Position / Proportion  = How Far away we are
      Integral
      Derivative

 */