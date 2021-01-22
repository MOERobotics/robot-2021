package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.GenericRobot;

public class DriveStraightFourFeet extends GenericAutonomous {
      double startingDistance;

      @Override
      protected void printSmartDashboardInternal() {
            SmartDashboard.putNumber("StartingDistance", startingDistance);
      }

      @Override public void autonomousPeriodic(GenericRobot robot) {
            switch (autonomousStep) {
                  case 0:
                        startingDistance = robot.getDistanceInchesLeft();
                        autonomousStep = 1;
                  case 1:
                        robot.driveForward(0.3);
                        double currentDistance = robot.getDistanceInchesLeft();
                        if (currentDistance - startingDistance  > 48) {
                              autonomousStep = 2;
                        }
                        break;
                  case 2:
                        robot.driveForward(0);
                        break;
            }
      }
}
