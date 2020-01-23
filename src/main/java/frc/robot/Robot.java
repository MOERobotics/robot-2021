/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.GenericAutonomous;
import frc.robot.autonomous.PlanA;
import frc.robot.autonomous.Win;
import frc.robot.genericrobot.*;

public class Robot extends TimedRobot {

    GenericAutonomous autoProgram = new Win();
    GenericAutonomous betterAuto = new PlanA();
    GenericRobot robot = new KeerthanPracticeOne();
    Joystick leftJoystick = new Joystick(0);
    double deadZone = 0.1;
    Lidar lidar = new Lidar();

  @Override public void robotInit() {
    lidar.start();

  }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("Left  Encoder Ticks", robot.getDistanceTicksLeft());
        SmartDashboard.putNumber("Right Encoder Ticks", robot.getDistanceTicksRight());
        SmartDashboard.putNumber("Navx Yaw", robot.getYaw());
        SmartDashboard.putNumber("Navx Pitch", robot.getPitch());
        SmartDashboard.putNumber("Navx Roll", robot.getRoll());

        SmartDashboard.putNumber("Left  Motor Power", robot.getMotorPowerLeft());
        SmartDashboard.putNumber("Right Motor Power", robot.getMotorPowerRight());
        SmartDashboard.putNumber("Upper Shooter Power", robot.getShooterPowerUpper());
        SmartDashboard.putNumber("Lower Shooter Power", robot.getShooterPowerLower());
        SmartDashboard.putNumber("Control Panel Power", robot.getControlPanelSpinnerPower());

        SmartDashboard.putNumber("AutoStep", betterAuto.autonomousStep);
        SmartDashboard.putBoolean("Shifter state", robot.getShifterState());
        SmartDashboard.putNumber("Left Encoder Inches", robot.getDistanceInchesLeft());
        SmartDashboard.putNumber("Right Encoder Inches", robot.getDistanceInchesRight());

        SmartDashboard.putBoolean("Lidar Locked", lidar.isLocked());

        for(int i = 0; i <= 3; i++) {
            Integer lidarNum = lidar.getDistance(i);
            String lidarString = " ";
            if (lidarNum != null) {
                lidarString = lidarNum.toString();
            }
            SmartDashboard.putString("Lidar" + i, lidarString);
        }
    }

    @Override
    public void disabledPeriodic() {
        if (leftJoystick.getTriggerPressed()) {
            System.out.println("AAAAAAAA");
            robot.resetAttitude();
            robot.resetEncoders();
        }

        betterAuto.autonomousStep = 0;

    }

    @Override
    public void autonomousInit() {
        betterAuto.autonomousInit(robot);
    }

    @Override
    public void autonomousPeriodic() {
        betterAuto.autonomousPeriodic(robot);
    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {
        double leftPower = -leftJoystick.getY() + leftJoystick.getX();
        double rightPower = -leftJoystick.getY() - leftJoystick.getX();

        if (leftPower < -deadZone) {
            leftPower = (leftPower + deadZone) / (1 - deadZone);
        } else if (leftPower > deadZone) {
            leftPower = (leftPower - deadZone) / (1 - deadZone);
        } else {
            leftPower = 0;
        }

        if (rightPower < -deadZone) {
            rightPower = (rightPower + deadZone) / (1 - deadZone);
        } else if (rightPower > deadZone) {
            rightPower = (rightPower - deadZone) / (1 - deadZone);
        } else {
            rightPower = 0;
        }

        robot.setMotorPowerPercentage(leftPower, rightPower);
        robot.setShooterPowerPercentage(0);

        if (leftJoystick.getRawButtonPressed(16)) {
            robot.shiftLow();
        }

        if (leftJoystick.getRawButtonPressed(11)) {
            robot.shiftHigh();
        }
        if (leftJoystick.getRawButton(13)) {
            robot.driveForward(-.2);
        }
        if (leftJoystick.getRawButton(14)) {
            robot.driveForward(.2);
        }

    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {

    }

}
