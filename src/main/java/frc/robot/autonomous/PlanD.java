package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PIDModule;
import frc.robot.genericrobot.GenericRobot;

public class PlanD extends GenericAutonomous {

      //change speed depending on robot!! (CaMOElot = .4, TestBot = .2)
      double defaultSpeed = 0.2;

      static double startingYaw = 0.0; //start at an angle, figure out later
      static double startingDistance = 0.0;
      PIDModule PIDSteering = new PIDModule(4.0e-2, 0.0e-3, 1.0e-4);
      double correction;
      static double currentYaw = 0;
      double outerArcLength = 80;
      double innerArc = 35.45;
      double innerRadius = 30;
      double outerRadius = 82; //decrease to around 78ish
      double yawDifference = 0;
      long startingTime = System.currentTimeMillis();
      double prevStartingDistance = 0;

      @Override
      public void autonomousInit(GenericRobot robot) {
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
                        PIDSteering.resetError();
                        currentYaw = 0;
                        autonomousStep = 1;
                        break;
                  case 1: //straightaway
                        PIDSteering.sendError(robot.getYaw() - currentYaw);
                        correction = PIDSteering.getCorrection();
                        robot.setMotorPowerPercentage(defaultSpeed * (1 + correction), defaultSpeed * (1 - correction));
                        currentDistance = robot.getDistanceInchesLeft();
                        if (currentDistance - startingDistance > 80) { //possibly extend
                              robot.driveForward(0);
                              autonomousStep = 2;
                        }
                        break;
                  case 2: //reset for backward straight-away
                        startingDistance = robot.getDistanceInchesLeft();
                        PIDSteering.resetError();
                        currentYaw = 0;
                        autonomousStep = 3;
                        break;
                  case 3: //backward straight-away
                        PIDSteering.sendError(robot.getYaw() - currentYaw);
                        correction = PIDSteering.getCorrection();
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
                        PIDSteering.resetError();
                        startingYaw = robot.getYaw();
                        autonomousStep = 5;
                        break;
                  case 5: //left arc to pick up third ball and two on other side
                        yawDifference = (robot.getYaw() - startingYaw) / 180 * Math.PI;
                        PIDSteering.sendError((robot.getDistanceInchesRight() - startingDistance) + outerRadius * yawDifference);
                        SmartDashboard.putNumber("Pid heading", (robot.getDistanceInchesRight() - startingDistance) + outerRadius * yawDifference);
                        correction = PIDSteering.getCorrection();
                        robot.setMotorPowerPercentage((defaultSpeed * .75) * (1 + correction), (defaultSpeed * 1.5) * (1 - correction));
                        currentDistance = robot.getDistanceInchesRight();
                        if (currentDistance - startingDistance > outerArcLength) {
                              autonomousStep = 6;
                        }
                        break;








                        /*
                        case 6: //reset for inverse arc (not resetting starting distance)
                        PIDSteering.resetError();
                        startingYaw = robot.getYaw();
                        autonomousStep = 7;
                        prevStartingDistance = startingDistance;
                        startingDistance = robot.getDistanceInchesRight();
                        break;

                            case 7: //backwards arc to previous position
                        yawDifference = (robot.getYaw() - startingYaw) / 180 * Math.PI;
                        PIDSteering.sendError((robot.getDistanceInchesRight() - startingDistance) + outerRadius * yawDifference);
                        SmartDashboard.putNumber("Pid heading", (robot.getDistanceInchesRight() - startingDistance) + outerRadius * yawDifference);
                        correction = PIDSteering.getCorrection();
                        robot.setMotorPowerPercentage((defaultSpeed * -.75) * (1 - correction), (defaultSpeed * -1.5) * (1 + correction));
                        currentDistance = robot.getDistanceInchesRight();
                        if (currentDistance - prevStartingDistance <= 0) {
                              autonomousStep = 8;
                        }
                        break;
                         */

                  case 6: //stop, hopefully auto aim to target later
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