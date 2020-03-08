package frc.robot.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.commands.GenericCommand;
import frc.robot.commands.LimelightAlign;
import frc.robot.genericrobot.GenericRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Util.*;

public class PlanC extends GenericAutonomous {

      //change speed depending on robot!! (CaMOElot = .4, TestBot = .2)
      double defaultSpeed = 0.25;

      static double startingYaw = 0.0; //start at an angle, figure out later
      static double startingDistance = 0.0;
      double correction;
      static double currentYaw = 0;
      double outerArcLength = 50;
      double innerArc = 35.45;
      double innerRadius = 30;
      double outerRadius = 65;
      double yawDifference = 0;
      double prevStartingDistance = 0;
      long startingTime = System.currentTimeMillis();
      int ballCount = 0;
      boolean shooting = false;
      double escalatorPower;
      double indexerPower;
      long alignWait = 2000;
      GenericCommand activeCommand = new LimelightAlign( -0.5, .8); //planA set setPoint to -2
      CollectPowerCells getCells = new CollectPowerCells();

      @Override
      public void autonomousInit(GenericRobot robot) {
            startingTime = System.currentTimeMillis();
            autonomousStep = -1;
            getCells.begin(robot);
      }

      @Override
      public void autonomousPeriodic(GenericRobot robot) {
            PIDController PIDSteering = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());

            double currentDistance = 0;
            double yawError;
            switch (autonomousStep) {
                  case -1: //resets and waits
                        defaultSpeed = 0.1;
                        ballCount = 0;
                        shooting = false;
                        robot.setShooterSpeedPresetName(GenericRobot.ShooterSpeedPresetName.SHORT_RANGE);
                        robot.setShooterRPMFromSpeedConst();
                        robot.resetAttitude();
                        robot.resetEncoders();
                        if (System.currentTimeMillis() >= startingTime + 100) {
                              autonomousStep += 1;
                        }
                        break;

                  case 0: //turns on LEDs
                        robot.limelight.table.getEntry("ledMode").setNumber(3);
                        robot.limelight.table.getEntry("pipeline").setNumber(0);

                        activeCommand.begin(robot);
                        activeCommand.setEnabled(true);
                        startingTime = System.currentTimeMillis();
                        autonomousStep += 1;
                        break;

                  case 1: //auto aligns
                        if (activeCommand.isEnabled() && ((System.currentTimeMillis() - startingTime) < alignWait)) {
                              activeCommand.step(robot);

                        } else {
                              robot.limelight.table.getEntry("ledMode").setNumber(1);
                              autonomousStep += 1;
                        }
                        break;

                  case 2: //you may fire when ready
                        if (robot.readyToShoot()) {
                              escalatorPower = 0.5;
                              indexerPower = 1.0;
                        } else {
                              escalatorPower = 0.0;
                              indexerPower = 0.0;
                        }
                        if (robot.getEscalatorSensorHigh() == true) {
                              shooting = true;
                        }
                        if ((shooting) && (robot.getEscalatorSensorHigh() == false)) {
                              shooting = false;
                              ballCount++;
                        }
                        if (ballCount == 3) {
                              escalatorPower = 0;
                              indexerPower = 0;
                              autonomousStep += 1;
                        }
                        robot.escalatorUp(escalatorPower);
                        robot.indexerLoad(indexerPower);
                        break;

                  case 3: //PID reset for straightaway
                        getCells.run(robot);
                        startingDistance = robot.getDistanceInchesLeft();
                        PIDSteering.reset();
                        PIDSteering.enableContinuousInput(-180, 180);
                        currentYaw = 0;
                        autonomousStep += 1;
                        break;

                  case 4: //straightaway linear deceleration
                        getCells.run(robot);

                        double scaleDown = speedScale(0, 86, 1.2, 1, currentDistance-startingDistance);
                        correction = PIDSteering.calculate(robot.getYaw() - currentYaw);
                        robot.setMotorPowerPercentage(scaleDown * defaultSpeed * (1 + correction), scaleDown * defaultSpeed * (1 - correction));
                        currentDistance = robot.getDistanceInchesLeft();
                        if (currentDistance - startingDistance > 86) { //maybe change depending on how far we need to go
                              autonomousStep += 1;
                        }
                        break;

                  case 5: //straightaway constant speed
                        getCells.run(robot);

                        correction = PIDSteering.calculate(robot.getYaw() - currentYaw);
                        robot.setMotorPowerPercentage(defaultSpeed * (1 + correction),
                                defaultSpeed * (1 - correction));
                        currentDistance = robot.getDistanceInchesLeft();
                        if (currentDistance - startingDistance > 104) { //maybe change depending on how far we need to go
                              robot.driveForward(0);
                              autonomousStep += 1;
                        }
                        break;

                  case 6: //reset for backward straight-away
                        getCells.run(robot);

                        startingDistance = robot.getDistanceInchesLeft();
                        PIDSteering.reset();
                        PIDSteering.enableContinuousInput(-180, 180);
                        currentYaw = 0;
                        autonomousStep += 1;
                        break;

                  case 7: //backward straight-away
                        getCells.run(robot);

                        correction = PIDSteering.calculate(robot.getYaw() - currentYaw);
                        robot.setMotorPowerPercentage(-1.3 * defaultSpeed * (1 - correction),
                                -1.3 * defaultSpeed * (1 + correction));
                        currentDistance = robot.getDistanceInchesLeft();
                        if (currentDistance - startingDistance < -64) { //maybe change depending on how far we need to go
                              robot.driveForward(0);
                              autonomousStep = 14;
                        }
                        break;

                  case 8: //reset for arc
                        getCells.run(robot);

                        startingDistance = robot.getDistanceInchesRight();
                        PIDSteering.reset();
                        PIDSteering.disableContinuousInput();
                        startingYaw = robot.getYaw();
                        autonomousStep += 1;
                        break;

                  case 9: //left arc to pick up third ball
                        getCells.run(robot);

                        yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                        correction = PIDSteering.calculate((robot.getDistanceInchesRight() - startingDistance) + outerRadius * yawDifference);
                        robot.setMotorPowerPercentage((defaultSpeed * .75) * (1 + correction), (defaultSpeed * 1.5) * (1 - correction));
                        currentDistance = robot.getDistanceInchesRight();
                        if (currentDistance - startingDistance > outerArcLength) {
                              autonomousStep += 1;
                        }
                        break;

                  case 10: //reset for inverse arc (not resetting starting distance)
                        getCells.run(robot);

                        PIDSteering.reset();
                        PIDSteering.disableContinuousInput();
                        startingYaw = robot.getYaw();
                        prevStartingDistance = startingDistance;
                        startingDistance = robot.getDistanceInchesRight();
                        autonomousStep += 1;
                        break;

                  case 11: //backwards arc to previous position
                        getCells.run(robot);

                        yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                        correction = PIDSteering.calculate((robot.getDistanceInchesRight() - startingDistance) + outerRadius * yawDifference);
                        robot.setMotorPowerPercentage((defaultSpeed * -.75) * (1 - correction), (defaultSpeed * -1.5) * (1 + correction));
                        currentDistance = robot.getDistanceInchesRight();
                        if (currentDistance - prevStartingDistance <= 0) {
                              autonomousStep += 1;
                        }
                        break;
                  case 12: // continue collecting and start timer
                        getCells.run(robot);
                        startingTime = System.currentTimeMillis();
                        autonomousStep += 1;
                        break;

                  case 13: // continue collecting for 2 seconds
                        getCells.run(robot);
                        long currentTime = System.currentTimeMillis();
                        if ((currentTime - startingTime) > 0) {
                              autonomousStep += 1;
                              break;
                        }

                  case 14: // stop collecting
                        getCells.stop(robot);
                        robot.driveForward(0);
                        ballCount = 0;
                        autonomousStep += 1;
                        break;

                  case 15: // align
                        robot.limelight.table.getEntry("ledMode").setNumber(3);
                        robot.limelight.table.getEntry("pipeline").setNumber(0);
                        activeCommand = new LimelightAlign(-3, .8); //fix dem bois
                        activeCommand.begin(robot);
                        activeCommand.setEnabled(true);
                        ballCount = 0;
                        startingTime = System.currentTimeMillis();
                        autonomousStep += 1;
                        break;

                  case 16:
                        if (activeCommand.isEnabled() && ((System.currentTimeMillis() - startingTime) < alignWait)) {
                              activeCommand.step(robot);

                        } else {
                              robot.limelight.table.getEntry("ledMode").setNumber(1);
                              autonomousStep += 1;
                        }
                        break;

                  case 17: // you may fire when ready
                        if (robot.readyToShoot()) {
                              escalatorPower = 0.5;
                              indexerPower = 1.0;
                        } else {
                              escalatorPower = 0.0;
                              indexerPower = 0.0;
                        }
                        if (robot.getEscalatorSensorHigh() == true) {
                              shooting = true;
                        }
                        if ((shooting) && (robot.getEscalatorSensorHigh() == false)) {
                              shooting = false;
                              ballCount++;
                        }
                        if (ballCount == 2) {
                              escalatorPower = 0;
                              indexerPower = 0;
                              autonomousStep += 1;
                        }
                        robot.escalatorUp(escalatorPower);
                        robot.indexerLoad(indexerPower);
                        break;

                  case 18: //cease your autonomous
                        robot.setShooterPowerPercentage(0);
                        if (activeCommand.isEnabled()) {
                              activeCommand.step(robot);

                        } else {
                              robot.limelight.table.getEntry("ledMode").setNumber(1);
                              robot.driveForward(0);
                        }
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