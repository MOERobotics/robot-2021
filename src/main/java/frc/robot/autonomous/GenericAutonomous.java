package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Logger;
import frc.robot.genericrobot.GenericRobot;

import static frc.robot.Util.normalizeAngleRadians;

public abstract class GenericAutonomous {

      public int autonomousStep = 0;

      /* A routine to make angle differences map to a continuous domain [-Pi,Pi].*/
      public double continuousAngleDiff(double theta) {
            return normalizeAngleRadians(theta);
      }


      public final void printSmartDashboard() {
            SmartDashboard.putNumber ("Autonomous Step"    ,            autonomousStep);
            SmartDashboard.putString ("Autonomous Program" , this.getClass().getName());
            printSmartDashboardInternal();
      }

      protected void printSmartDashboardInternal() {

      }
      public void autonomousInit(GenericRobot robot) {
            autonomousStep = 0;
            Logger.log("AUTOINIT", "I don't have autonomousInit in my autonomous program :'(");
      }

      public void autonomousPeriodic(GenericRobot robot) {
            Logger.log("AUTOPERIODIC","I don't have autonomousPeriodic in my autonomous program :'(");
      }
}
