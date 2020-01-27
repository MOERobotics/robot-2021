package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PIDModule;
import frc.robot.genericrobot.GenericRobot;

public class Win extends GenericAutonomous {

      //change speed depending on robot!! (CaMOElot = .4, TestBot = .2)
      double defaultSpeed = 0.2;

      double startingYaw      = 0.0;
      double startingDistance = 0.0;
      PIDModule PIDSteering = new PIDModule(4.0e-2, 0.0e-3, 1.0e-4);
      double correction;
      static double currentYaw = 0;
      double leftWheelArc = (Math.PI * 46) / 2;
      double rightWheelArc = (Math.PI * 23 / 2);

      @Override public void autonomousInit(GenericRobot robot) {
            robot.resetAttitude();
            robot.resetEncoders();
            autonomousStep = 0;
      }

      @Override public void autonomousPeriodic(GenericRobot robot) {
            double currentDistance = 0;
            switch (autonomousStep) {
                case 0:                             //shorten steps to implement S curve B)
                        if (true) autonomousStep = 2;
                        break;
                  case 1:
                  case 2:
                        startingYaw = robot.getYaw();
                        autonomousStep = 3;
                  case 3:
                        robot.driveLeftInPlace(defaultSpeed);
                        currentYaw = robot.getYaw();
                        if (currentYaw - startingYaw < -90) {
                              robot.driveForward(0);
                              autonomousStep = 4;
                        } else break;
                  case 4:
                        startingDistance = robot.getDistanceInchesLeft();
                        PIDSteering.resetError(); //reset pid stuff
                        currentYaw = -90;
                        autonomousStep = 5;
                        break;
                  case 5:
                        PIDSteering.sendError(robot.getYaw()-currentYaw);
                        correction = PIDSteering.getCorrection();
                        robot.setMotorPowerPercentage(defaultSpeed *(1+correction), defaultSpeed *(1-correction));
                        currentDistance = robot.getDistanceInchesLeft();
                        if (currentDistance - startingDistance > 18.5) { //drive towards wall also was 34.5
                              robot.driveForward(0);
                              autonomousStep = 6;
                        } else break;
                  case 6:
                        PIDSteering.resetError();
                        startingYaw = robot.getYaw();
                        startingDistance = robot.getDistanceInchesLeft(); //check
                        autonomousStep = 7;
                  case 7:
                        PIDSteering.sendError(robot.getDistanceInchesLeft()/robot.getDistanceInchesRight()-2.0);
                        correction = PIDSteering.getCorrection();
                        robot.setMotorPowerPercentage((defaultSpeed * 1.5) *(1+correction), (defaultSpeed * .75)*(1-correction));
                        currentDistance = robot.getDistanceInchesLeft();
                        if (currentDistance - startingDistance > leftWheelArc) {
                              robot.driveForward(0);
                              autonomousStep = 8;
                        } else break;
                        currentYaw = robot.getYaw();

                  case 8:
                        startingDistance = robot.getDistanceInchesLeft();
                        PIDSteering.resetError();
                        currentYaw = 0;
                        autonomousStep = 9;

                  case 9:
                        PIDSteering.sendError(robot.getYaw() - currentYaw);
                        correction = PIDSteering.getCorrection();
                        robot.setMotorPowerPercentage(defaultSpeed *(1+correction), defaultSpeed *(1-correction));
                        currentDistance = robot.getDistanceInchesLeft();
                        if (currentDistance - startingDistance > 135) {
                              robot.driveForward(0);
                              autonomousStep = 10;
                        } else break;
                  case 10:
                        robot.driveForward(0);
                        //                               ¯\_(ツ)_/¯
                        break;
            }
            SmartDashboard.putNumber("Correction",correction);
      }
}



/*

      Position / Proportion  = How Far away we are
      Integral
      Derivative

      wheel to wheel: 23in

 */