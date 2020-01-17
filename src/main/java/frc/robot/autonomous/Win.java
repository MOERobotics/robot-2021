package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.PIDModule;
import frc.robot.genericrobot.GenericRobot;

public class Win extends GenericAutonomous {

      double startingYaw      = 0.0;
      double startingDistance = 0.0;
      PIDModule PIDSteering = new PIDModule(4.0e-2, 0.0e-3, 1.0e-4);
      double correction;
      static double currentYaw = 0;
      double leftWheelArc = (Math.PI * 46) / 2;
      double rightWheelArc = (Math.PI * 23 / 2);

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
                        if (currentDistance - startingDistance > 34.5) { //drive towards wall
                              robot.driveForward(0);
                              autonomousStep = 6;
                        } else break;
                  case 6:
                        PIDSteering.resetError();
                        startingYaw = robot.getYaw();
                        startingDistance = robot.getDistanceTicksLeft(); //check
                        autonomousStep = 7;
                  case 7:
                        PIDSteering.setHeading(robot.getDistanceInchesLeft()/robot.getDistanceInchesRight()-2.0);
                        correction = PIDSteering.getCorrection();
                        robot.setMotorPowerPercentage(0.6*(1+correction), 0.3*(1-correction));
                        currentDistance = robot.getDistanceInchesLeft();
                        if (currentDistance - startingDistance > leftWheelArc) {
                              robot.driveForward(0);
                              autonomousStep = 6;
                        } else break;
                        //robot.driveRightInPlace(0.4);
                        currentYaw = robot.getYaw();
                        /*
                              if (currentYaw - startingYaw > 85) {
                              robot.driveForward(0);
                              autonomousStep = 8;
                        } else break;
                         */


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
            SmartDashboard.putNumber("Correction",correction);
      }
}



/*

      Position / Proportion  = How Far away we are
      Integral
      Derivative

      wheel to wheel: 23in

 */