/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.GenericAutonomous;
import frc.robot.autonomous.Win;
import frc.robot.genericrobot.GenericRobot;
import frc.robot.genericrobot.KeerthanPracticeOne;
import frc.robot.genericrobot.Lidar;

public class Robot extends TimedRobot {

  GenericAutonomous autoProgram = new Win();
  GenericRobot robot = new KeerthanPracticeOne();
  Joystick leftJoystick = new Joystick(0);
  Lidar lidar = new Lidar();

  @Override public void robotInit() {
    lidar.start();

  }

  @Override public void robotPeriodic() {
    SmartDashboard.putNumber("Left  Encoder Ticks", robot.getDistanceTicksLeft ());
    SmartDashboard.putNumber("Right Encoder Ticks", robot.getDistanceTicksRight());
    SmartDashboard.putNumber("Navx Yaw"           , robot.getYaw  ());
    SmartDashboard.putNumber("Navx Pitch"         , robot.getPitch());
    SmartDashboard.putNumber("Navx Roll"          , robot.getRoll ());

    SmartDashboard.putNumber("Left  Motor Power"  , robot.getMotorPowerLeft ());
    SmartDashboard.putNumber("Right Motor Power"  , robot.getMotorPowerRight());
    SmartDashboard.putNumber("Upper Shooter Power", robot.getShooterPowerUpper());
    SmartDashboard.putNumber("Lower Shooter Power", robot.getShooterPowerLower());
    SmartDashboard.putNumber("Control Panel Power", robot.getControlPanelSpinnerPower());

    SmartDashboard.putNumber("AutoStep", autoProgram.autonomousStep);

  for(int i = 1; i <= 4; i++) {
    Integer lidar1 = lidar.getDistance(i);
    String lidar1s = " ";
    if (lidar1 != null) {
      lidar1s = lidar1.toString();
    }
    SmartDashboard.putString("Lidar" + i, lidar1s);
  }
  }

  @Override public void disabledPeriodic() {
    if (leftJoystick.getTriggerPressed()) {
      System.out.println("AAAAAAAA");
      robot.resetAttitude();
    }
  }

  @Override public void autonomousInit() {
    autoProgram.autonomousInit(robot);
  }

  @Override public void autonomousPeriodic() {
    autoProgram.autonomousPeriodic(robot);
  }

  @Override public void teleopInit() {

  }

  @Override public void teleopPeriodic() {
    double leftPower  = -leftJoystick.getY() + leftJoystick.getX();
    double rightPower = -leftJoystick.getY() - leftJoystick.getX();


    leftPower  = ( leftPower < 0.1 &&  leftPower > -0.1) ? 0 :  leftPower;
    rightPower = (rightPower < 0.1 && rightPower > -0.1) ? 0 : rightPower;

    robot.setMotorPowerPercentage(leftPower,rightPower);
    robot.setShooterPowerPercentage(0);
  }

  @Override public void testInit() {

  }

  @Override public void testPeriodic() {

  }

}
