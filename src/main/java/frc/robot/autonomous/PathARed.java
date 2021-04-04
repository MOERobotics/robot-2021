package frc.robot.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.GenericRobot;


import java.lang.Math;

// robot starts on start zone on the c1 line
public class PathARed extends GenericAutonomous {

    PIDController myPID = new PIDController(0,0,0);
    CollectPowerCells getCells = new CollectPowerCells();

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

    double leftPower = 0;
    double rightPower = 0;

    double dist1 = 54;
    double dist2 = 30*Math.sqrt(5);
    double dist3 = 30*Math.sqrt(10);
    double dist4 = 150;

    double ang1 = 0;
    double ang2 = Math.toDegrees(Math.atan(1.0/2.0));
    double ang3 = -1*Math.toDegrees(Math.atan(3.0));
    double ang4 = 0;

    double dir1Left = 0;
    double dir1Right = 0;

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
        if (startingYaw < 0){
            startingYaw +=360;
        }
        autonomousStep = -1;
        desired_distance = 60;
        desired_yaw = 90;
        myPID = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
        myPID.reset();
        myPID.enableContinuousInput(-180,180);

    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {

        switch (autonomousStep) {
            case -1: //reset and wait
                robot.resetAttitude();
                robot.resetEncoders();
                if (System.currentTimeMillis() >= startingTime + 100) {
                    start_inches = robot.getDistanceInchesLeft();
                    startingYaw = robot.getYaw();
                    if (startingYaw < 0){
                        startingYaw +=360;
                    }
                    autonomousStep += 1;
                }
                getCells.run(robot);
                break;
            case 0:
                //turn
                getCells.run(robot);
                leftPower = dir1Left;
                rightPower = dir1Right;

                desired_yaw = Math.abs(ang1);

                currYaw = robot.getYaw();
                if (currYaw < 0) {
                    currYaw += 360;
                }
                yawChange = Math.abs(currYaw-startingYaw);
                yawChange = Math.min(yawChange, 360 - yawChange);
                if (yawChange >= desired_yaw) {

                    leftPower = 0;
                    rightPower = 0;

                    start_inches = robot.getDistanceInchesLeft();
                    startingYaw = ang1;
                    if (startingYaw< 0){
                        startingYaw+= 360;
                    }
                    myPID.reset();
                    myPID.enableContinuousInput(-180, 180);
                    autonomousStep += 1;
                }

                break;
            case 1:
                //drive
                getCells.run(robot);
                correction = myPID.calculate(robot.getYaw() - startingYaw);

                leftPower = default_speed + correction;
                rightPower = default_speed - correction;

                desired_distance = dist1;

                inches_traveled = (robot.getDistanceInchesLeft() - start_inches);
                if (inches_traveled >= desired_distance) {

                    leftPower = 0;
                    rightPower = 0;

                    start_inches = robot.getDistanceInchesLeft();
                    startingYaw = robot.getYaw();
                    if (startingYaw < 0){
                        startingYaw += 360;
                    }
                    myPID.reset();
                    myPID.enableContinuousInput(-180, 180);
                    autonomousStep += 1;
                }

                break;
            case 2:
                //turn
                getCells.run(robot);
                leftPower = dir2Left;
                rightPower = dir2Right;

                desired_yaw = Math.abs(ang1) + Math.abs(ang2);

                currYaw = robot.getYaw();
                if (currYaw < 0) {
                    currYaw += 360;
                }
                yawChange = Math.abs(currYaw-startingYaw);
                yawChange = Math.min(yawChange, 360 - yawChange);
                if (yawChange >= desired_yaw) {

                    leftPower = 0;
                    rightPower = 0;

                    start_inches = robot.getDistanceInchesLeft();
                    startingYaw = ang2;
                    if (startingYaw < 0){
                        startingYaw += 360;
                    }
                    myPID.reset();
                    myPID.enableContinuousInput(-180, 180);
                    autonomousStep += 1;
                }

                break;
            case 3:
                //drive
                getCells.run(robot);
                correction = myPID.calculate(robot.getYaw() - startingYaw);

                desired_distance = dist2;

                leftPower = default_speed + correction;
                rightPower = default_speed - correction;

                inches_traveled = (robot.getDistanceInchesLeft() - start_inches);
                if (inches_traveled >= desired_distance) {

                    leftPower = 0;
                    rightPower = 0;

                    start_inches = robot.getDistanceInchesLeft();
                    startingYaw = robot.getYaw();
                    if (startingYaw < 0){
                        startingYaw += 360;
                    }
                    myPID.reset();
                    myPID.enableContinuousInput(-180, 180);
                    autonomousStep += 1;
                }

                break;
            case 4:
                //turn
                getCells.run(robot);
                leftPower = dir3Left;
                rightPower = dir3Right;

                desired_yaw = Math.abs(ang3)+Math.abs(ang2);

                currYaw = robot.getYaw();
                if (currYaw < 0) {
                    currYaw += 360;
                }
                yawChange = Math.abs(currYaw-startingYaw);
                yawChange = Math.min(yawChange, 360 - yawChange);
                if (yawChange >= desired_yaw) {

                    leftPower = 0;
                    rightPower = 0;

                    start_inches = robot.getDistanceInchesLeft();
                    startingYaw = ang3;
                    if (startingYaw < 0){
                        startingYaw += 360;
                    }
                    myPID.reset();
                    myPID.enableContinuousInput(-180, 180);
                    autonomousStep += 1;
                }

                break;
            case 5:
                //drive
                getCells.run(robot);
                correction = myPID.calculate(robot.getYaw() - startingYaw);

                leftPower = default_speed + correction;
                rightPower = default_speed - correction;

                desired_distance = dist3;

                inches_traveled = (robot.getDistanceInchesLeft() - start_inches);
                if (inches_traveled >= desired_distance) {

                    leftPower = 0;
                    rightPower = 0;

                    start_inches = robot.getDistanceInchesLeft();
                    startingYaw = robot.getYaw();
                    if (startingYaw < 0){
                        startingYaw += 360;
                    }
                    myPID.reset();
                    myPID.enableContinuousInput(-180, 180);
                    autonomousStep += 1;
                }

                break;
            case 6:
                //turn
                getCells.run(robot);
                leftPower = dir4Left;
                rightPower = dir4Right;

                desired_yaw = Math.abs(ang3)+Math.abs(ang4);

                currYaw = robot.getYaw();
                if (currYaw < 0) {
                    currYaw += 360;
                }
                yawChange = Math.abs(currYaw-startingYaw);
                yawChange = Math.min(yawChange, 360 - yawChange);
                if (yawChange >= desired_yaw) {

                    leftPower = 0;
                    rightPower = 0;

                    start_inches = robot.getDistanceInchesLeft();
                    startingYaw = ang4;
                    if (startingYaw < 0){
                        startingYaw += 360;
                    }
                    myPID.reset();
                    myPID.enableContinuousInput(-180, 180);
                    autonomousStep += 1;
                }

                break;
            case 7:
                //drive
                getCells.run(robot);
                correction = myPID.calculate(robot.getYaw() - startingYaw);

                leftPower = default_speed + correction;
                rightPower = default_speed - correction;

                desired_distance = dist4;

                inches_traveled = (robot.getDistanceInchesLeft() - start_inches);
                if (inches_traveled >= desired_distance) {

                    leftPower = 0;
                    rightPower = 0;
                    getCells.stop(robot);

                    start_inches = robot.getDistanceInchesLeft();
                    startingYaw = robot.getYaw();
                    if (startingYaw < 0){
                        startingYaw += 360;
                    }
                    myPID.reset();
                    myPID.enableContinuousInput(-180, 180);
                    autonomousStep += 1;
                }

                break;

        }

        robot.setMotorPowerPercentage(leftPower,rightPower);


    }
}