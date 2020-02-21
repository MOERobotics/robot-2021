package frc.robot.genericrobot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Camoelot extends GenericRobot {

      AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 50);

      private TalonSRX  leftMotorA = new TalonSRX(12);
      private TalonSRX  leftMotorB = new TalonSRX(13);
      private TalonSRX  leftMotorC = new TalonSRX(14);
      private TalonSRX rightMotorA = new TalonSRX( 1);
      private TalonSRX rightMotorB = new TalonSRX( 2);
      private TalonSRX rightMotorC = new TalonSRX( 3);

      private TalonSRX shooterA = new TalonSRX(15);
      private TalonSRX shooterB = new TalonSRX(10);

      private Encoder  leftEncoder = new Encoder(0,1);
      private Encoder rightEncoder = new Encoder(2,3);

      public Camoelot () {
            rightMotorA.setInverted(true);
            rightMotorB.setInverted(true);
            rightMotorC.setInverted(true);
      }

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
            return navx.getYaw();
      }

      @Override
      public double getPitch() {
            return navx.getPitch();
      }

      @Override
      public double getRoll() {
            return navx.getRoll();
      }

      @Override
      protected void setShooterPowerPercentageInternal(double upperPower, double lowerPower) {
            shooterA.set(ControlMode.PercentOutput, upperPower * 0.40);
            shooterB.set(ControlMode.PercentOutput, lowerPower * 0.40);
      }

      @Override
      public double getDistanceTicksLeft() {
            return leftEncoder.get();
      }

      @Override
      public double getDistanceTicksRight() {
            return -rightEncoder.get();
      }

      @Override
      public double getDistanceRatioLeft() {
            return 116;
      }

      @Override
      public double getDistanceRatioRight() {
            return 116;
      }

      @Override public void resetAttitude() {
            navx.reset();
      }

      @Override
      public double getLimelightMinpower() {
            return 0.4;
      }
}
