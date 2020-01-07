package frc.robot.genericrobot;

import edu.wpi.first.wpilibj.*;
import static jdk.jshell.spi.ExecutionControl.NotImplementedException;

public abstract class GenericRobot {

      public double  leftPower = 0;
      public double rightPower = 0;

      public final void setMotorPowerPercentage(
            double leftPower,
            double rightPower
      ) {
            this. leftPower =  leftPower;
            this.rightPower = rightPower;
            setMotorPowerPercentageInternal(
                    leftPower,
                    rightPower
            );
      }
      protected abstract void setMotorPowerPercentageInternal(
              double leftPower,
              double rightPower
      );

      public void driveForward (
            double power
      ) {
            setMotorPowerPercentage(power,power);
      }

      //TODO: allan please add details
      public void avoidWW3Draft() throws NotImplementedException {
            throw new RuntimeException("AAAAAAAA");
      }

      public double getMotorPowerLeft() {
            return leftPower;
      }



      public double getDistanceInchesLeft()  {
            System.out.println("I don't have an encoder :'(");
            return 0;
      }

      public double getDistanceInchesRight()  {
            System.out.println("I don't have an encoder :'(");
            return 0;
      }




}
