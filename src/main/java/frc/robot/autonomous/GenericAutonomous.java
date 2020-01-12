package frc.robot.autonomous;

import frc.robot.genericrobot.GenericRobot;

public abstract class GenericAutonomous {

      public int autonomousStep = 0;

      public void autonomousInit(GenericRobot robot) {
            autonomousStep = 0;
            System.out.println("I don't have autonomousInit in my autonomous program :'(");
      }

      public void autonomousPeriodic(GenericRobot robot) {
            System.out.println("I don't have autonomousPeriodic in my autonomous program :'(");
      }
}
