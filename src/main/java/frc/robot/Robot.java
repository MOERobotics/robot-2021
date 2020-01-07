/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.*;

public class Robot extends TimedRobot {

  GenericRobot steve = new Camoelot();
  Joystick leftJoystick = new Joystick(0);

  @Override public void robotInit() {

  }

  @Override public void robotPeriodic() {
    SmartDashboard.putNumber("DistanceInches", steve.getDistanceInchesLeft());
    SmartDashboard.putNumber("Left Motor Power", steve.getMotorPowerLeft());
  }

  @Override public void autonomousInit() {

  }

  @Override public void autonomousPeriodic() {

  }

  @Override public void teleopInit() {

  }

  @Override public void teleopPeriodic() {
    double leftPower  = leftJoystick.getY() + leftJoystick.getX();
    double rightPower = leftJoystick.getY() - leftJoystick.getX();

    steve.setMotorPowerPercentage(leftPower,rightPower);
  }

  @Override public void testInit() {

  }

  @Override public void testPeriodic() {

  }

}
