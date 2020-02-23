package frc.robot.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.commands.GenericCommand;
import frc.robot.commands.LimelightAlign;
import frc.robot.genericrobot.GenericRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
      double outerRadius = 50;
      double yawDifference = 0;
      double prevStartingDistance = 0;
      long startingTime = System.currentTimeMillis();
      int ballCount = 0;
      boolean shooting = false;
      double escalatorPower;
      double indexerPower;
      double shooterUpperRPMNear = 2210;
      double shooterLowerRPMNear = 2210;
      double shooterUpperRPMFar = 2430;
      double shooterLowerRPMFar = 2430;
      GenericCommand activeCommand = new LimelightAlign( -4, .8, .0185); //planA set setPoint to -2
      CollectPowerCells getCells = new CollectPowerCells();

      @Override
      public void autonomousInit(GenericRobot robot) {
            startingTime = System.currentTimeMillis();
            autonomousStep = 0;
            getCells.begin(robot);
      }

      @Override
      public void autonomousPeriodic(GenericRobot robot) {
            PIDController PIDSteering = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());

            double currentDistance = 0;
            double yawError;
            switch (autonomousStep) {
                  case -1: //resets and waits
                        ballCount = 0;
                        shooting = false;
                        robot.setShooterRPM(shooterUpperRPMNear, shooterLowerRPMNear);
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
                        autonomousStep += 1;
                        break;

                  case 1: //auto aligns
                        if (activeCommand.isEnabled()) {
                              activeCommand.step(robot);

                        } else {
                              robot.limelight.table.getEntry("ledMode").setNumber(1);
                              autonomousStep += 1;
                        }
                        break;

                  case 2:
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
                        startingDistance = robot.getDistanceInchesLeft();
                        PIDSteering.reset();
                        PIDSteering.enableContinuousInput(-180, 180);
                        currentYaw = 0;
                        autonomousStep += 1;
                        break;

                  case 4: //straightaway
                        getCells.run(robot);

                        correction = PIDSteering.calculate(robot.getYaw() - currentYaw);
                        robot.setMotorPowerPercentage(defaultSpeed * (1 + correction), defaultSpeed * (1 - correction));
                        currentDistance = robot.getDistanceInchesLeft();
                        if (currentDistance - startingDistance > 80) { //maybe change depending on how far we need to go
                              robot.driveForward(0);
                              autonomousStep += 1;
                        }
                        break;

                  case 5: //reset for backward straight-away
                        getCells.run(robot);

                        startingDistance = robot.getDistanceInchesLeft();
                        PIDSteering.reset();
                        PIDSteering.enableContinuousInput(-180, 180);
                        currentYaw = 0;
                        autonomousStep += 1;
                        break;

                  case 6: //backward straight-away
                        getCells.run(robot);

                        correction = PIDSteering.calculate(robot.getYaw() - currentYaw);
                        robot.setMotorPowerPercentage(-1 * defaultSpeed * (1 - correction), -1 * defaultSpeed * (1 + correction));
                        currentDistance = robot.getDistanceInchesLeft();
                        SmartDashboard.putNumber("startDistance", startingDistance);
                        SmartDashboard.putNumber("currentDistance", currentDistance);
                        SmartDashboard.putNumber("distanceDifference", currentDistance - startingDistance);
                        if (currentDistance - startingDistance < -40) { //maybe change depending on how far we need to go
                              robot.driveForward(0);
                              autonomousStep += 1;
                        }
                        break;

                  case 7: //reset for arc
                        getCells.run(robot);

                        startingDistance = robot.getDistanceInchesRight();
                        PIDSteering.reset();
                        PIDSteering.disableContinuousInput();
                        startingYaw = robot.getYaw();
                        autonomousStep += 1;
                        break;

                  case 8: //left arc to pick up third ball
                        getCells.run(robot);

                        yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                        correction = PIDSteering.calculate((robot.getDistanceInchesRight() - startingDistance) + outerRadius * yawDifference);
                        robot.setMotorPowerPercentage((defaultSpeed * .75) * (1 + correction), (defaultSpeed * 1.5) * (1 - correction));
                        currentDistance = robot.getDistanceInchesRight();
                        if (currentDistance - startingDistance > outerArcLength) {
                              autonomousStep += 1;
                        }
                        break;

                  case 9: //reset for inverse arc (not resetting starting distance)
                        getCells.run(robot);

                        PIDSteering.reset();
                        PIDSteering.disableContinuousInput();
                        startingYaw = robot.getYaw();
                        prevStartingDistance = startingDistance;
                        startingDistance = robot.getDistanceInchesRight();
                        autonomousStep += 1;
                        break;

                  case 10: //backwards arc to previous position
                        getCells.run(robot);

                        yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                        correction = PIDSteering.calculate((robot.getDistanceInchesRight() - startingDistance) + outerRadius * yawDifference);
                        robot.setMotorPowerPercentage((defaultSpeed * -.75) * (1 - correction), (defaultSpeed * -1.5) * (1 + correction));
                        currentDistance = robot.getDistanceInchesRight();
                        if (currentDistance - prevStartingDistance <= 0) {
                              autonomousStep += 1;
                        }
                        break;
                  case 11:
                        getCells.run(robot);
                        startingTime = System.currentTimeMillis();
                        autonomousStep += 1;
                        break;

                  case 12:
                        getCells.run(robot);
                        long currentTime = System.currentTimeMillis();
                        if ((currentTime - startingTime) > 2000) {
                              autonomousStep += 1;
                              break;
                        }

                  case 13:
                        getCells.stop(robot);
                        robot.driveForward(0);
                        ballCount = 0;
                        autonomousStep += 1;
                        break;

                  case 14:
                        robot.limelight.table.getEntry("ledMode").setNumber(3);
                        robot.limelight.table.getEntry("pipeline").setNumber(1);
                        activeCommand.setEnabled(true);
                        ballCount = 0;
                        autonomousStep += 1;
                        break;

                  case 15:
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

                  case 16: //cease your autonomous
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