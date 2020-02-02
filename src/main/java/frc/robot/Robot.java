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

    // WheelOfFortune colorWheel = new WheelOfFortune();
    GenericAutonomous autoProgram = new Win();
    GenericRobot robot = new KeerthanPracticeOne();
    Joystick leftJoystick = new Joystick(0);
    double deadZone = 0.1;
    Lidar lidar = new Lidar();

    @Override public void robotInit() {
        lidar.start();
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber  ("Left  Encoder Ticks"  , robot.getDistanceTicksLeft()        );
        SmartDashboard.putNumber  ("Right Encoder Ticks"  , robot.getDistanceTicksRight()       );
        SmartDashboard.putNumber  ("Navx Yaw"             , robot.getYaw()                      );
        SmartDashboard.putNumber  ("Navx Pitch"           , robot.getPitch()                    );
        SmartDashboard.putNumber  ("Navx Roll"            , robot.getRoll()                     );

        SmartDashboard.putNumber  ("Left  Motor Power"    , robot.getMotorPowerLeft()           );
        SmartDashboard.putNumber  ("Right Motor Power"    , robot.getMotorPowerRight()          );
        SmartDashboard.putNumber  ("Upper Shooter Power"  , robot.getShooterPowerUpper()        );
        SmartDashboard.putNumber  ("Lower Shooter Power"  , robot.getShooterPowerLower()        );
        SmartDashboard.putNumber  ("Control Panel Power"  , robot.getControlPanelSpinnerPower() );

        SmartDashboard.putNumber  ("AutoStep"             , autoProgram.autonomousStep          );
        SmartDashboard.putString  ("Shifter state"        , robot.getShifterState().toString()  );
        SmartDashboard.putNumber  ("Left Encoder Inches"  , robot.getDistanceInchesLeft()       );
        SmartDashboard.putNumber  ("Right Encoder Inches" , robot.getDistanceInchesRight()      );

        SmartDashboard.putBoolean ( "Lidar Locked"        , lidar.isLocked()                    );

        //SmartDashboard.putString("Instant Color", colorWheel.getAndStoreInstantColor().toString());
        //SmartDashboard.putString("Inferred Color",  colorWheel.getInferredColor().toString());


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

    public double deadzoneValue(double input, double deadZone) {
        if (input < -deadZone) {
            return (input + deadZone) / (1 - deadZone);
        } else if (input > deadZone) {
            return (input - deadZone) / (1 - deadZone);
        } else {
            return 0;
        }
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

    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {

    }

}
