package frc.robot.autonomous;

import frc.robot.PIDModule;
import frc.robot.genericrobot.GenericRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PlanC extends GenericAutonomous {

      //change speed depending on robot!! (CaMOElot = .4, TestBot = .2)
      double defaultSpeed = 0.2;

      static double startingYaw = 0.0; //start at an angle, figure out later
      static double startingDistance = 0.0;
      PIDModule PIDSteering = new PIDModule(4.0e-2, 0.0e-3, 1.0e-4);
      double correction;
      static double currentYaw = 0;
      double outerArc = 20;
      double innerArc = 35.45;
      double innerRadius = 30;
      double outerRadius = 70;
      double yawDifference = 0;
      long startingTime = System.currentTimeMillis();

      @Override
      public void autonomousInit(GenericRobot robot) {
            autonomousStep = 2;
      }

      @Override
      public void autonomousPeriodic(GenericRobot robot) {
            double currentDistance = 0;
            double yawError;
            switch (autonomousStep) {

                  case -1: //resets everything and waits
                        robot.resetAttitude();
                        robot.resetEncoders();
                        if (System.currentTimeMillis() >= startingTime + 100) {
                              autonomousStep = 0;
                        }
                        break;
                  case 0: //PID reset for straightaway
                        startingDistance = robot.getDistanceInchesLeft();
                        PIDSteering.resetError();
                        currentYaw = 0;
                        autonomousStep = 1;
                        break;
                  case 1:
                        PIDSteering.sendError(robot.getYaw() - currentYaw);
                        correction = PIDSteering.getCorrection();
                        robot.setMotorPowerPercentage(defaultSpeed * (1 + correction), defaultSpeed * (1 - correction));
                        currentDistance = robot.getDistanceInchesLeft();
                        if (currentDistance - startingDistance > 80) { //maybe change depending on how far we need to go
                              robot.driveForward(0);
                              autonomousStep = 2;
                        } else break;
                  case 2:
                        startingDistance = robot.getDistanceInchesRight();
                        PIDSteering.resetError();
                        currentYaw = 0;
                        autonomousStep = 3;
                        break;
                  case 3:
                        PIDSteering.sendError(robot.getDistanceInchesLeft() / robot.getDistanceInchesRight() - 0.5); //-2 is A
                        correction = PIDSteering.getCorrection();
                        robot.setMotorPowerPercentage((defaultSpeed * -.75) * (1 + correction), (defaultSpeed * -1.5) * (1 - correction));
                        currentDistance = robot.getDistanceInchesRight();
                        SmartDashboard.putNumber("startDistance", startingDistance);
                        SmartDashboard.putNumber("currentDistance", currentDistance);
                        SmartDashboard.putNumber("distanceDifference", currentDistance - startingDistance);
                        if (currentDistance - startingDistance < -outerArc) {
                              autonomousStep = 4;
                        }
                        break;
                  case 4:
                        startingDistance = robot.getDistanceInchesRight();
                        PIDSteering.resetError();
                        currentYaw = 0;
                        autonomousStep = 4;
                        break;
                  case 5:
                        PIDSteering.sendError(robot.getDistanceInchesLeft() / robot.getDistanceInchesRight() - 0.5); //-2 is A
                        correction = PIDSteering.getCorrection();
                        robot.setMotorPowerPercentage((defaultSpeed * .75) * (1 + correction), (defaultSpeed * 1.5) * (1 - correction));
                        currentDistance = robot.getDistanceInchesRight();
                        SmartDashboard.putNumber("startDistance", startingDistance);
                        SmartDashboard.putNumber("currentDistance", currentDistance);
                        SmartDashboard.putNumber("distanceDifference", currentDistance - startingDistance);
                        if (currentDistance - startingDistance < -outerArc) {
                              autonomousStep = 6;
                        }
                        break;
                  case 6:
                        robot.driveForward(0);
                        //                               ¯\_(ツ)_/¯
                        break;



//¯\_(ツ)_/¯
                  /*

                  case -1: //resets everything and waits
                        robot.resetAttitude();
                        robot.resetEncoders();
                        if (System.currentTimeMillis() >= startingTime + 100) {
                              autonomousStep = 0;
                        }
                  case 0: //PID reset for 1st (left) arc
                        PIDSteering.resetError();
                        startingYaw = robot.getYaw();
                        startingDistance = robot.getDistanceInchesRight();
                        autonomousStep = 1;
                        break;
                  case 1:
                        PIDSteering.sendError(robot.getDistanceInchesLeft() / robot.getDistanceInchesRight() - 0.5); //-2 is A
                        correction = PIDSteering.getCorrection();
                        robot.setMotorPowerPercentage((defaultSpeed * .75) * (1 + correction), (defaultSpeed * 1.5) * (1 - correction));
                        currentDistance = robot.getDistanceInchesRight();
                        if (currentDistance - startingDistance > outerArc) {
                              autonomousStep = 2;
                        }
                        break;
                  case 2: //PID reset for 2nd (right) arc
                        PIDSteering.resetError();
                        startingDistance = robot.getDistanceInchesLeft();
                        startingYaw = robot.getYaw();
                        autonomousStep = 3;
                  case 3:
                        PIDSteering.sendError(robot.getDistanceInchesLeft() / robot.getDistanceInchesRight() - 2.0);
                        correction = PIDSteering.getCorrection();
                        robot.setMotorPowerPercentage((defaultSpeed * 1.5) * (1 + correction), (defaultSpeed * .75) * (1 - correction));
                        currentDistance = robot.getDistanceInchesLeft();
                        if (currentDistance - startingDistance > outerArc) {
                              autonomousStep = 4;
                        }
                        break;
                  case 4: //PID reset for straightaway
                        startingDistance = robot.getDistanceInchesLeft();
                        PIDSteering.resetError();
                        currentYaw = 0;
                        autonomousStep = 5;
                  case 5:
                        PIDSteering.sendError(robot.getYaw() - currentYaw);
                        correction = PIDSteering.getCorrection();
                        robot.setMotorPowerPercentage(defaultSpeed * (1 + correction), defaultSpeed * (1 - correction));
                        currentDistance = robot.getDistanceInchesLeft();
                        if (currentDistance - startingDistance > 100) { //maybe change depending on how far we need to go
                              robot.driveForward(0);
                              autonomousStep = 6;
                        } else break;
                  case 6:
                        robot.driveForward(0);
                        //                               ¯\_(ツ)_/¯
                        break;

            }
                   */

            }
      }
}
/*

      Position / Proportion  = How Far away we are
      Integral
      Derivative

      wheel to wheel: 23in

 */