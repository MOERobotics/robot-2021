/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.autonomous.*;
import frc.robot.genericrobot.*;
import static frc.robot.Util.*;

public class Robot extends TimedRobot {

    //WheelOfFortune    colorWheel   = new WheelOfFortune();
    GenericAutonomous autoProgram  = new DriveStraightOneSecond(); //Auto routine to be used?
    GenericRobot      robot        = new Falcon();
    Joystick          leftJoystick = new Joystick(0);
    Joystick          xboxJoystick = new Joystick(0); //to be changed

    double            deadZone     = 0.10;

    @Override public void robotInit() {}

    @Override
    public void robotPeriodic() {
        robot      .printSmartDashboard();
        autoProgram.printSmartDashboard();

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

        //Collector
       // if (xboxJoystick.
    }


    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {

        double leftPower = -leftJoystick.getY() + leftJoystick.getX();
        double rightPower = -leftJoystick.getY() - leftJoystick.getX();

        leftPower = deadzoneValue( leftPower,deadZone);
        rightPower = deadzoneValue(rightPower,deadZone);

        robot.setMotorPowerPercentage(leftPower, rightPower);

        //Collector
        if (leftJoystick.getRawButton(11)) {
            robot.collectorIn(.2);
        } else if (leftJoystick.getRawButton(16)) {
            robot.collectorOut(.2);
        } else {
            robot.setCollectorPower(0);
        }

        //Escalator
        if (leftJoystick.getRawButton(12)) {
            robot.escalatorUp(.5);
        } else if (leftJoystick.getRawButton(15)) {
            robot.escalatorDown(.5);
        } else {
            robot.setEscalatorPower(0);
        }

        //Shooter
        if (leftJoystick.getRawButton(13)) {
            robot.setShooterPowerPercentage(1.0);
        } else if (leftJoystick.getRawButton(14)) {
            robot.setShooterPowerPercentage(0);
        }

        //Indexer
        if (leftJoystick.getRawButton( 7)) {
            robot.indexerLoad(1.0);
        } else if (leftJoystick.getRawButton( 8)) {
            robot.indexerUnload(1.0);
        } else {
            robot.setIndexerPower(0);
        }

        //Vert Adjust
        if (leftJoystick.getRawButton( 6)) {
            robot.aimUp(.2);
        } else if (leftJoystick.getRawButton( 9)) {
            robot.aimDown(.2);
        } else {
            robot.setAngleAdjusterPower(0);
        }

        //CP
        if (leftJoystick.getRawButton( 5)) {
            robot.spinControlPanel(-.2);
        } else if (leftJoystick.getRawButton(10)) {
            robot.spinControlPanel(.2);
        } else {
            robot.spinControlPanel(0);
        }

        //Climb vert
        if (leftJoystick.getRawButton( 2)) {
            robot.climbDown(.2);
        } else if (leftJoystick.getRawButton( 1)) {
            robot.climbUp(.2);
        } else {
            robot.climbVertical(0);
        }

        //climb horiz
        if (leftJoystick.getRawButton( 3)) {
            robot.climberBalanceLeft(-.2);
        } else if (leftJoystick.getRawButton( 4)) {
            robot.climberBalanceRight(.2);
        } else {
            robot.setBalancePower(0);
        }

    }

}
