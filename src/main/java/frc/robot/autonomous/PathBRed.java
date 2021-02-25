package frc.robot.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.GenericRobot;


import java.lang.Math;

// robot starts on start zone on the c1 line
public class PathBRed extends GenericAutonomous {

    PIDController myPID = new PIDController(0,0,0);

    double inches_traveled = 0;
    double desired_distance = 60;
    double desired_yaw = 0;
    double default_speed = .2;
    double start_inches = 0;
    double startingYaw = 0;
    double currYaw = 0;
    double yawChange = 0;
    int autonomousStep = 0;
    double correction = 0;
    double startingTime = 0;

    double dist1 = 30*Math.sqrt(5);
    double dist2 = 60*Math.sqrt(2);
    double dist3 = 60*Math.sqrt(2);
    double dist4 = 120;

    double ang1 = Math.toDegrees(Math.atan(1.0/2.0));
    double ang2 = 45 + Math.toDegrees(Math.atan(1.0/2.0));
    double ang3 = 90;
    double ang4 = 45;

    double dir1Left = -.2;
    double dir1Right = .2;

    double dir2Left = .2;
    double dir2Right = -.2;

    double dir3Left = -.2;
    double dir3Right = .2;

    double dir4Left = .2;
    double dir4Right = -.2;


    @Override
    protected void printSmartDashboardInternal(){
        SmartDashboard.putNumber("Autonomous Step", autonomousStep);

        SmartDashboard.putNumber("Desired Yaw", desired_yaw);
        SmartDashboard.putNumber("Starting Yaw", startingYaw);
        SmartDashboard.putNumber("Yaw Change", currYaw);

        SmartDashboard.putNumber("Inches Traveled", inches_traveled);
        SmartDashboard.putNumber("Start Ticks", start_inches);
        SmartDashboard.putNumber("Desired Distance", desired_distance);

        SmartDashboard.putNumber("PID correction", correction);

    }
    @Override
    public void autonomousInit(GenericRobot robot){
        startingTime = System.currentTimeMillis();
        start_inches = robot.getDistanceInchesLeft();
        startingYaw = robot.getYaw();
        autonomousStep = -1;
        desired_distance = 60;
        desired_yaw = 90;
        myPID = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
        myPID.reset();
        myPID.enableContinuousInput(-180,180);

    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        inches_traveled = (robot.getDistanceInchesLeft() - start_inches);
        if (inches_traveled >= desired_distance) {
            robot.setMotorPowerPercentage(0, 0);
            start_inches = robot.getDistanceInchesLeft();
            startingYaw = robot.getYaw();
            if (startingYaw < 0){
                startingYaw += 360;
            }
            myPID.reset();
            myPID.enableContinuousInput(-180, 180);
            autonomousStep += 1;
        }

        currYaw = robot.getYaw();
        if (currYaw < 0) {
            currYaw += 360;
        }
        yawChange = Math.abs(currYaw-startingYaw);
        yawChange = Math.min(yawChange, 360 - yawChange);
        if (yawChange >= desired_yaw) {
            robot.setMotorPowerPercentage(0, 0);
            start_inches = robot.getDistanceInchesLeft();
            startingYaw = robot.getYaw();
            if (startingYaw < 0){
                startingYaw += 360;
            }
            myPID.reset();
            myPID.enableContinuousInput(-180, 180);
            autonomousStep += 1;
        }

        switch (autonomousStep) {
            case -1: // reset and wait
                robot.resetAttitude();
                robot.resetEncoders();
                if (System.currentTimeMillis() >= startingTime + 100) {
                    autonomousStep += 1;
                }
                break;
            case 0:
                //turn
                robot.setMotorPowerPercentage(dir1Left,dir1Right);
                desired_yaw = ang1;
                break;
            case 1:
                //drive
                correction = myPID.calculate(robot.getYaw() - startingYaw);
                robot.setMotorPowerPercentage(default_speed*(1+correction), default_speed*(1-correction));
                desired_distance = dist1;
                break;
            case 2:
                //turn
                robot.setMotorPowerPercentage(dir2Left,dir2Right);
                desired_yaw = ang2;
                break;
            case 3:
                //drive
                correction = myPID.calculate(robot.getYaw() - startingYaw);
                robot.setMotorPowerPercentage(default_speed*(1+correction), default_speed*(1-correction));
                desired_distance = dist2;
                break;
            case 4:
                //turn
                robot.setMotorPowerPercentage(dir3Left,dir3Right);
                desired_yaw = ang3;
                break;
            case 5:
                //drive
                correction = myPID.calculate(robot.getYaw() - startingYaw);
                robot.setMotorPowerPercentage(default_speed*(1+correction), default_speed*(1-correction));
                desired_distance = dist3;
                break;
            case 6:
                //turn
                robot.setMotorPowerPercentage(dir4Left,dir4Right);
                desired_yaw = ang4;
                break;
            case 7:
                //drive
                correction = myPID.calculate(robot.getYaw() - startingYaw);
                robot.setMotorPowerPercentage(default_speed*(1+correction), default_speed*(1-correction));
                desired_distance = dist4;
                break;

        }


    }
}