package frc.robot.autonomous;

import frc.robot.genericrobot.GenericRobot;

public abstract class GenericAutonomous {

      public int autonomousStep = 0;

      /* A routine to make angle differences map to a continuous domain [-Pi,Pi].*/
      public double continuousAngleDiff(double theta) {
            if (theta > Math.PI) {
                  theta -= 2*Math.PI;
            }
            if (theta < -Math.PI) {
                  theta += 2*Math.PI;
            }
            return theta;
      }

      public void autonomousInit(GenericRobot robot) {
            autonomousStep = 0;
            System.out.println("I don't have autonomousInit in my autonomous program :'(");
      }

      public void autonomousPeriodic(GenericRobot robot) {
            System.out.println("I don't have autonomousPeriodic in my autonomous program :'(");
      }
}
