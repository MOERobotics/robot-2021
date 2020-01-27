package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PIDModule;
import frc.robot.genericrobot.GenericRobot;

public class PlanA extends GenericAutonomous {

      //change speed depending on robot!! (CaMOElot = .4, TestBot = .2)
      double defaultSpeed = 0.3;

      static double startingYaw      = 0.0;
      static double startingDistance = 0.0;
      PIDModule PIDSteering = new PIDModule(4.0e-2, 0.0e-3, 1.0e-4);
      double correction;
      static double currentYaw = 0;
      double outerArc = 73.2;
      double innerArc = 35.45;
      double outerRadius = 70;
      double yawDifference = 0;
      long startingTime = System.currentTimeMillis();

      @Override public void autonomousInit(GenericRobot robot) {
            autonomousStep = -1;
      }

      @Override public void autonomousPeriodic(GenericRobot robot) {
            double currentDistance = 0;
            double yawError;
            switch (autonomousStep) {

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
      }}

/*

      Position / Proportion  = How Far away we are
      Integral
      Derivative

      wheel to wheel: 23in

 */