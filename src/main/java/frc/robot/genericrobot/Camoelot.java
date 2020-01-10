package frc.robot.genericrobot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Camoelot extends GenericRobot {

      AHRS navx;
      private TalonSRX  leftMotorA = new TalonSRX(12);
      private TalonSRX  leftMotorB = new TalonSRX(13);
      private TalonSRX  leftMotorC = new TalonSRX(14);
      private TalonSRX rightMotorA = new TalonSRX( 1);
      private TalonSRX rightMotorB = new TalonSRX( 2);
      private TalonSRX rightMotorC = new TalonSRX( 3);
      private TalonSRX shooterA = new TalonSRX(15);
      private TalonSRX shooterB = new TalonSRX(10);



      private Encoder  leftEncoder = new Encoder(0,1);
      private Encoder rightEncoder = new Encoder(0,1);

      @Override public void setMotorPowerPercentageInternal(
              double leftPower,
              double rightPower
      ) {
             leftMotorA.set(ControlMode.PercentOutput,  leftPower);
             leftMotorB.set(ControlMode.PercentOutput,  leftPower);
             leftMotorC.set(ControlMode.PercentOutput,  leftPower);
            rightMotorA.set(ControlMode.PercentOutput, rightPower);
            rightMotorB.set(ControlMode.PercentOutput, rightPower);
            rightMotorC.set(ControlMode.PercentOutput, rightPower);
      }




      @Override
      public double getYaw() {
            navx.getYaw();
            return 0;
      }

      @Override
      public double getPitch() {
            navx.getPitch();
            return 0;
      }

      @Override
      public double getRole() {
            navx.getRoll();
            return 0;
      }

      @Override
      protected void setShooterPowerPercentageInternal(double upperPower, double lowerPower) {
            shooterA.set(ControlMode.PercentOutput, upperPower);
            shooterB.set(ControlMode.PercentOutput, lowerPower);
      }



}
