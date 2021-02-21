package frc.robot.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.GenericRobot;
import java.lang.Math;

// robot starts in 33.7 degree angle to the x-axis, facing SE. Robot breaks plane of A1B1 in start zone\
public class GalacticB extends GenericAutonomous {

    PIDController myPID = new PIDController(0,0,0);

    double inches_traveled = 0;
    double desired_distance = 72;
    double desired_yaw = 0;
    boolean cellPathCheck = true;
    boolean PathARed = false;
    boolean PathABlue = false;
    boolean PathBBlue = false;
    boolean PathBRed = false;
    double default_speed = .2;
    double start_inches = 0;
    double startingYaw = 0;
    double currYaw = 0;
    double yawChange = 0;
    int autonomousStep = 0;
    double correction = 0;
    boolean drive = false;

    double dist1 = 0;
    double dist2 = 0;
    double dist3 = 0;
    double dist4 = 0;

    double ang1 = 0;
    double ang2 = 0;
    double ang3 = 0;
    double ang4 = 0;

    double dir1Left = .2;
    double dir1Right = .2;
    double dir2Left = .2;
    double dir2Right = .2;
    double dir3Left = .2;
    double dir3Right = .2;
    double dir4Left = .2;
    double dir4Right = .2;


    @Override
    protected void printSmartDashboardInternal(){
        SmartDashboard.putNumber("Autonomous Step", autonomousStep);

        SmartDashboard.putBoolean("PathA red", PathARed);
        SmartDashboard.putBoolean("PathA blue", PathABlue);
        SmartDashboard.putBoolean("PathB red", PathBRed);
        SmartDashboard.putBoolean("PathB blue", PathBBlue);

        SmartDashboard.putBoolean("cell check", cellPathCheck);
        SmartDashboard.putBoolean("drive", drive);

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
        start_inches = robot.getDistanceInchesLeft();
        startingYaw = robot.getYaw();
        autonomousStep = 0;
        PathARed = false;
        PathABlue = false;
        cellPathCheck = true;
        desired_distance = 20*Math.sqrt(13);
        desired_yaw = 90;
        myPID = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
        myPID.reset();
        myPID.enableContinuousInput(-180,180);

    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {

        if (cellPathCheck){
            inches_traveled = (robot.getDistanceInchesLeft() - start_inches);
            if (inches_traveled >= desired_distance) {
                robot.setMotorPowerPercentage(0, 0);
                start_inches = robot.getDistanceInchesLeft();
                startingYaw = robot.getYaw();
                if (startingYaw < 0) {
                    startingYaw += 360;
                }
                myPID.reset();
                myPID.enableContinuousInput(-180, 180);
                autonomousStep += 1;
            }

            switch (autonomousStep){
                case 0:
                    correction = myPID.calculate(robot.getYaw() - startingYaw);
                    robot.setMotorPowerPercentage(default_speed*(1+correction), default_speed*(1-correction));
                    desired_distance = 60;
                    break;
                case 1:
                    if (cellThere()) {
                        PathARed = true;

                        dist1 = 30;
                        dist2 = 30*Math.sqrt(5);
                        dist3 = 30*Math.sqrt(10);
                        dist4 = 150;

                        ang1 = 90;
                        ang2 = 180-Math.toDegrees(Math.atan(2));
                        ang3 = Math.toDegrees(Math.atan(3)+Math.atan(1/2));
                        ang4 = Math.toDegrees(Math.atan(3));

                        // robot turns left/right/left/right
                        dir1Left *= -1;
                        dir2Right *= -1;
                        dir3Left *= -1;
                        dir4Right *= -1;

                        autonomousStep = 0;
                        cellPathCheck = false;
                        drive = true;
                    }
                    else {
                        autonomousStep += 1;
                    }
                    break;
                case 2:
                    correction = myPID.calculate(robot.getYaw() - startingYaw);
                    robot.setMotorPowerPercentage(default_speed*(1+correction), default_speed*(1-correction));
                    desired_distance = 60;

                    break;
                case 3:
                    if (cellThere()) {
                        PathBRed = true;

                        dist1 = 60*Math.sqrt(2);
                        dist2 = 240;
                        dist3 = 0;
                        dist4 = 0;

                        ang1 = 135;
                        ang2 = 135;
                        ang3 = 0;
                        ang4 = 0;

                        //robot turns left/right
                        dir1Left *= -1;
                        dir2Right *= -1;

                        autonomousStep = 0;
                        cellPathCheck = false;
                        drive = true;
                    }
                    else {
                        autonomousStep += 1;
                    }
                    break;
                case 4:
                    correction = myPID.calculate(robot.getYaw() - startingYaw);
                    robot.setMotorPowerPercentage(default_speed*(1+correction), default_speed*(1-correction));
                    desired_distance = 30;

                    break;
                case 5:
                    if (cellThere()) {
                        PathBBlue = true;

                        dist1 = 60*Math.sqrt(2);
                        dist2 = 60*Math.sqrt(2);
                        dist3 = 30;
                        dist4 = 0;

                        ang1 = 45;
                        ang2 = 90;
                        ang3 = 45;
                        ang4 = 0;

                        //robot turns left/right/left
                        dir1Left *= -1;
                        dir2Right *= -1;
                        dir3Left *= -1;

                        autonomousStep = 0;
                        cellPathCheck = false;
                        drive = true;
                    }
                    else {
                        PathABlue = true;

                        dist1 = 30;
                        dist2 = 30*Math.sqrt(10);
                        dist3 = 30*Math.sqrt(5);
                        dist4 = 60;

                        ang1 = 90;
                        ang2 = 90+Math.toDegrees(Math.atan(3));
                        ang3 = 180-(Math.toDegrees(Math.atan(1/3)+Math.atan(2)));
                        ang4 = Math.toDegrees(Math.atan(1/2));

                        // robot turns right/left/right/left

                        dir1Right *= -1;
                        dir2Left *= -1;
                        dir3Right *= -1;
                        dir4Left *= -1;

                        autonomousStep = 0;
                        cellPathCheck = false;
                        drive = true;
                    }
                    break;

            }
        }

        if (drive) {

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
    public boolean cellThere(){
        return Math.random() < 0.5;
    }

}
