package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.GenericRobot;
import edu.wpi.first.wpilibj.controller.PIDController;

public class PlanA extends GenericAutonomous {

      //change speed depending on robot!! (CaMOElot = .4, TestBot = .3)
      double defaultSpeed = 0.3;

      static double startingYaw      = 0.0;
      static double startingDistance = 0.0;
      PIDController PIDSteering = new PIDController(4.0e-2, 0.0e-3, 1.0e-4);
      double correction;
      static double currentYaw = 0;
      double outerArc = 73.2;
      double innerArc = 35.45;
      double outerRadius = 70;
      double yawDifference = 0;
      long startingTime;

      @Override public void autonomousInit(GenericRobot robot) {
            startingTime = System.currentTimeMillis();
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
                        break;

                  case 0: //PID reset for 1st (left) arc
                        PIDSteering.reset();
                        PIDSteering.disableContinuousInput();
                        startingYaw = robot.getYaw();
                        startingDistance = robot.getDistanceInchesRight();
                        autonomousStep = 1;
                        break;

                  case 1: //1st (left) arc
                        yawDifference = (robot.getYaw() - startingYaw) / 180 * Math.PI;
                        correction = PIDSteering.calculate((robot.getDistanceInchesRight() - startingDistance) + outerRadius * yawDifference);
                        SmartDashboard.putNumber("Pid heading", (robot.getDistanceInchesRight() - startingDistance) + outerRadius * yawDifference);
                        robot.setMotorPowerPercentage((defaultSpeed * .75) * (1 + correction), (defaultSpeed * 1.5) * (1 - correction));
                        currentDistance = robot.getDistanceInchesRight();
                        if (currentDistance - startingDistance > outerArc) {
                              autonomousStep = 2;
                        }
                        break;

                  case 2: //PID reset for 2nd (right) arc
                        PIDSteering.reset();
                        PIDSteering.disableContinuousInput();
                        startingDistance = robot.getDistanceInchesLeft();
                        startingYaw = robot.getYaw();
                        autonomousStep = 3;
                        break;

                  case 3: //2nd (right) arc
                        yawError = robot.getYaw() - startingYaw;
                        yawDifference = yawError*Math.PI*5.55555e-3;
                        correction = PIDSteering.calculate(outerRadius * yawDifference - (robot.getDistanceInchesLeft() - startingDistance));
                        robot.setMotorPowerPercentage((defaultSpeed * 1.5) * (1 + correction), (defaultSpeed * .75) * (1 - correction));
                        currentDistance = robot.getDistanceInchesLeft();
                        if(currentDistance - startingDistance > outerArc) {
                              autonomousStep = 4;
                        }
                        break;

                  case 4: //PID reset for straightaway
                        startingDistance = robot.getDistanceInchesLeft();
                        PIDSteering.reset();
                        PIDSteering.enableContinuousInput(-180,180);
                        currentYaw = 0;
                        autonomousStep = 5;
                        break;

                  case 5: //straightaway, a little bit of oscillation, may need to turn P & D - PID coefficients
                        correction = PIDSteering.calculate(robot.getYaw() - currentYaw);
                        robot.setMotorPowerPercentage(1.5 * defaultSpeed * (1 + correction), 1.5 * defaultSpeed * (1 - correction));
                        currentDistance = robot.getDistanceInchesLeft();
                        if (currentDistance - startingDistance > 100) {
                              robot.driveForward(0);
                              autonomousStep = 6;
                        }
                        break;

                  case 6: //cease your autnomous
                        robot.driveForward(0);
                        //                               ¯\_(ツ)_/¯
                        break;

            }
      }
}

