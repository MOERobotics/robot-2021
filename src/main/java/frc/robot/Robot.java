/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.*;
import frc.robot.commands.*;
import frc.robot.genericrobot.*;
import static frc.robot.Util.*;

public class Robot extends TimedRobot {

    //WheelOfFortune    colorWheel    = new WheelOfFortune();
    GenericAutonomous autoProgram   = new PlanA(); //Auto routine to be used?
    GenericCommand    activeCommand = GenericCommand.doNothingCommand;
    GenericRobot      robot         = new KeerthanPracticeOne();
    Joystick          leftJoystick  = new Joystick(0);
    double            deadZone      = 0.1;
    TrenchRun trenchRun = new TrenchRun(0);

    @Override public void robotInit() {}

    @Override
    public void robotPeriodic() {
        robot        .printSmartDashboard();
        autoProgram  .printSmartDashboard();
        activeCommand.printSmartDashboard();

        //SmartDashboard.putString("Instant Color", colorWheel.getAndStoreInstantColor().toString());
        //SmartDashboard.putString("Inferred Color",  colorWheel.getInferredColor().toString());
    }

    @Override
    public void disabledPeriodic() {
        if (leftJoystick.getTriggerPressed()) {
            System.out.println("AAAAAAAA");
            robot.resetAttitude();
            robot.resetEncoders();
        }
    }

    @Override
    public void autonomousInit() {
        autoProgram.autonomousInit(robot);
    }

    @Override
    public void autonomousPeriodic() {
        autoProgram.autonomousPeriodic(robot);
    }

    @Override
    public void teleopInit() {
    }


    @Override
    public void teleopPeriodic() {
        trenchRun = new TrenchRun(-leftJoystick.getY());
        if (leftJoystick.getRawButtonPressed(11)) {
            activeCommand.setEnabled(false);
        }

        if (activeCommand.isEnabled()) {
            activeCommand.step(robot);
            if (activeCommand.locksControls()) return;
        }
        double leftPower = -leftJoystick.getY() + leftJoystick.getX();
        double rightPower = -leftJoystick.getY() - leftJoystick.getX();

         leftPower = deadzoneValue( leftPower,deadZone);
        rightPower = deadzoneValue(rightPower,deadZone);

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
        if(leftJoystick.getRawButtonPressed(1)){
            trenchRun.setEnabled(true);
        }
    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {

    }

}
