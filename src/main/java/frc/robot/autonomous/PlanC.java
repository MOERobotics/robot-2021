package frc.robot.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.genericrobot.GenericRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PlanC extends GenericAutonomous {

      //change speed depending on robot!! (CaMOElot = .4, TestBot = .2)
      double defaultSpeed = 0.2;

      static double startingYaw = 0.0; //start at an angle, figure out later
      static double startingDistance = 0.0;
      PIDController PIDSteering = new PIDController(4.0e-2, 0.0e-3, 1.0e-4);
      double correction;
      static double currentYaw = 0;
      double outerArcLength = 50;
      double innerArc = 35.45;
      double innerRadius = 30;
      double outerRadius = 100;
      double yawDifference = 0;
      double prevStartingDistance = 0;
      long startingTime = System.currentTimeMillis();


      @Override
      public void autonomousInit(GenericRobot robot) {
            startingTime = System.currentTimeMillis();
            autonomousStep = -1;
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
                        PIDSteering.reset();
                        PIDSteering.enableContinuousInput(-180,180);
                        currentYaw = 0;
                        autonomousStep = 1;
                        break;

                  case 1: //straightaway
                        correction = PIDSteering.calculate(robot.getYaw() - currentYaw);
                        robot.setMotorPowerPercentage(defaultSpeed * (1 + correction), defaultSpeed * (1 - correction));
                        currentDistance = robot.getDistanceInchesLeft();
                        if (currentDistance - startingDistance > 80) { //maybe change depending on how far we need to go
                              robot.driveForward(0);
                              autonomousStep = 2;
                        }
                        break;

                  case 2: //reset for backward straight-away
                        startingDistance = robot.getDistanceInchesLeft();
                        PIDSteering.reset();
                        PIDSteering.enableContinuousInput(-180,180);
                        currentYaw = 0;
                        autonomousStep = 3;
                        break;

                  case 3: //backward straight-away
                        correction = PIDSteering.calculate(robot.getYaw() - currentYaw);
                        robot.setMotorPowerPercentage(-1 * defaultSpeed * (1 - correction), -1 * defaultSpeed * (1 + correction));
                        currentDistance = robot.getDistanceInchesLeft();
                        SmartDashboard.putNumber("startDistance", startingDistance);
                        SmartDashboard.putNumber("currentDistance", currentDistance);
                        SmartDashboard.putNumber("distanceDifference", currentDistance - startingDistance);
                        if (currentDistance - startingDistance < -40) { //maybe change depending on how far we need to go
                              robot.driveForward(0);
                              autonomousStep = 4;
                        }
                        break;

                  case 4: //reset for arc
                        startingDistance = robot.getDistanceInchesRight();
                        PIDSteering.reset();
                        PIDSteering.disableContinuousInput();
                        startingYaw = robot.getYaw();
                        autonomousStep = 5;
                        break;

                  case 5: //left arc to pick up third ball
                        yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                        correction = PIDSteering.calculate((robot.getDistanceInchesRight() - startingDistance) + outerRadius * yawDifference);
                        robot.setMotorPowerPercentage((defaultSpeed * .75) * (1 + correction), (defaultSpeed * 1.5) * (1 - correction));
                        currentDistance = robot.getDistanceInchesRight();
                        if (currentDistance - startingDistance > outerArcLength) {
                              autonomousStep = 6;
                        }
                        break;

                  case 6: //reset for inverse arc (not resetting starting distance)
                        PIDSteering.reset();
                        PIDSteering.disableContinuousInput();
                        startingYaw = robot.getYaw();
                        autonomousStep = 7;
                        prevStartingDistance = startingDistance;
                        startingDistance = robot.getDistanceInchesRight();
                        break;

                  case 7: //backwards arc to previous position
                        yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                        correction = PIDSteering.calculate((robot.getDistanceInchesRight() - startingDistance) + outerRadius * yawDifference);
                        robot.setMotorPowerPercentage((defaultSpeed * -.75) * (1 - correction), (defaultSpeed * -1.5) * (1 + correction));
                        currentDistance = robot.getDistanceInchesRight();
                        if (currentDistance - prevStartingDistance <= 0) {
                              autonomousStep = 8;
                        }
                        break;

                  case 8: //cease your autnomous
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

      wheel to wheel: 23in

 */