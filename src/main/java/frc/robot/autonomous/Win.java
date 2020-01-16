package frc.robot.autonomous;

import frc.robot.PIDModule;
import frc.robot.genericrobot.GenericRobot;

public class Win extends GenericAutonomous {

      double startingYaw      = 0.0;
      double startingDistance = 0.0;
      PIDModule PIDSteering = new PIDModule(1.0e-2, 1.0e-4, 0.0e-4);
      double correction;
      static double currentYaw = 0;

      @Override public void autonomousInit(GenericRobot robot) {
            robot.resetAttitude();
      }

      @Override public void autonomousPeriodic(GenericRobot robot) {
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
                        robot.driveLeftInPlace(0.4);
                        currentYaw = robot.getYaw();
                        if (currentYaw - startingYaw < -90) {
                              robot.driveForward(0);
                              autonomousStep = 4;
                        } else break;
                  case 4:
                        startingDistance = robot.getDistanceInchesLeft();
                        PIDSteering.resetError(); //reset pid stuff
                        currentYaw = -90;
                        // currentYaw = robot.getYaw();
                        autonomousStep = 5;
                        break;
                  case 5:
                        PIDSteering.setHeading(robot.getYaw()-currentYaw);
                        correction = PIDSteering.getCorrection();
                        robot.setMotorPowerPercentage(0.4*(1+correction), 0.4*(1-correction));
                        currentDistance = robot.getDistanceInchesLeft();
                        if (currentDistance - startingDistance > 66.91) {
                              robot.driveForward(0);
                              autonomousStep = 6;
                        } else break;
                  case 6:
                        startingYaw = robot.getYaw();
                        autonomousStep = 7;
                  case 7:
                        robot.driveRightInPlace(0.4);
                        currentYaw = robot.getYaw();
                        if (currentYaw - startingYaw > 85) {
                              robot.driveForward(0);
                              autonomousStep = 8;
                        } else break;
                  case 8:
                        startingDistance = robot.getDistanceInchesLeft();
                        PIDSteering.resetError();
                        currentYaw = 0;
                        // currentYaw = robot.getYaw();
                        autonomousStep = 9;

                  case 9:
                        PIDSteering.setHeading(robot.getYaw() - currentYaw);
                        correction = PIDSteering.getCorrection();
                        robot.setMotorPowerPercentage(0.4*(1+correction), 0.4*(1-correction));
                        //robot.driveForward(0.2);
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