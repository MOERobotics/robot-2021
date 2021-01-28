package frc.robot;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.GenericAutonomous;
import frc.robot.genericrobot.GenericRobot;
import java.lang.Math;


public class GalacticA extends GenericAutonomous {


    double inches_traveled = 0;
    double desired_distance = 72;
    double desired_yaw = 0;
    boolean begin = true;
    boolean cell_check = false;
    boolean red = false;
    boolean blue = false;
    double default_speed = .2;
    double start_ticks = 0;
    double startingYaw = 0;
    double yawChange = 0;
    int Case = 0;
    double placeholderForNow = 0;


    @Override
    protected void printSmartDashboardInternal(){
        SmartDashboard.putNumber("Turn", Case);

        SmartDashboard.putBoolean("red", red);
        SmartDashboard.putBoolean("blue", blue);
        SmartDashboard.putBoolean("begin", begin);
        SmartDashboard.putBoolean("cell check", cell_check);

        SmartDashboard.putNumber("inches_traveled", inches_traveled);
        SmartDashboard.putNumber("start ticks", start_ticks);
        SmartDashboard.putNumber("desired distance", desired_distance);

    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        if (begin){

            start_ticks = robot.getDistanceTicksLeft();
            begin = false;
            cell_check = true;
        }
        if (cell_check){
            robot.setMotorPowerPercentage(default_speed,default_speed);
            inches_traveled = (robot.getDistanceInchesLeft() - start_ticks); // subtract start distance to get distance travelled, divide by 116 to get distance in inches

            if (inches_traveled >= desired_distance) {
                robot.setMotorPowerPercentage(0,0);
                // check if power cell is there
                start_ticks = robot.getDistanceTicksLeft();
                startingYaw = robot.getYaw();

                if (cellThere()){
                    red = true;
                    desired_distance = 5*Math.sqrt(5)/2*12;
                    desired_yaw = 7.125;

                }
                else{
                    blue = true;
                    desired_distance = 5*Math.sqrt(13)/2*12;
                    desired_yaw = placeholderForNow;
                }


            }
        }
        if (red){
            inches_traveled = (robot.getDistanceInchesLeft() - start_ticks);
            if (inches_traveled >= desired_distance) {
                robot.setMotorPowerPercentage(0,0);
                start_ticks = robot.getDistanceInchesLeft();
                startingYaw = robot.getYaw();
                Case += 1;
            }
            yawChange = robot.getYaw()-startingYaw;
            if (yawChange<0){
                yawChange += 360;
            }
            yawChange = Math.min(yawChange, 360-yawChange);
            if (yawChange >= desired_yaw){
                collectBall();
                robot.setMotorPowerPercentage(0,0);
                startingYaw = robot.getYaw();
                start_ticks = robot.getDistanceInchesLeft();
                Case += 1;
            }

            switch (Case){
                case 0:
                    robot.setMotorPowerPercentage(-.2,.2);
                case 2:
                    robot.setMotorPowerPercentage(default_speed,default_speed);
                case 3:
                    robot.setMotorPowerPercentage(-.2,-.2);
                    desired_yaw = 81.87;
                case 4:
                    robot.setMotorPowerPercentage(default_speed,default_speed);
                    desired_distance = 5*Math.sqrt(10)/2*12;
                case 5:
                    robot.setMotorPowerPercentage(.2,-.2);
                    desired_yaw = placeholderForNow;
                case 6:
                    robot.setMotorPowerPercentage(default_speed,default_speed);
                    desired_distance = 25*6;
            }

        }
        if (blue){
            inches_traveled = (robot.getDistanceInchesLeft() - start_ticks);
            if (inches_traveled >= desired_distance) {
                start_ticks = robot.getDistanceInchesLeft();
                startingYaw = robot.getYaw();
                Case += 1;
            }
            yawChange = robot.getYaw()-startingYaw;
            if (yawChange<0){
                yawChange += 360;
            }
            yawChange = Math.min(yawChange, 360-yawChange);
            if (yawChange >= desired_yaw){
                collectBall();
                robot.setMotorPowerPercentage(0,0);
                start_ticks = robot.getDistanceInchesLeft();
                startingYaw = robot.getYaw();
                Case += 1;
            }
            switch (Case){
                case 0:
                    robot.setMotorPowerPercentage(default_speed,default_speed);
                case 1:
                    robot.setMotorPowerPercentage(-.2,.2);
                case 2:
                    robot.setMotorPowerPercentage(default_speed,default_speed);
                    desired_distance = 5 * Math.sqrt(10) / 2 * 12;
                case 3:
                    robot.setMotorPowerPercentage(.2,-.2);
                    desired_yaw = placeholderForNow;
                case 4:
                    robot.setMotorPowerPercentage(default_speed,default_speed);
                    desired_distance = 5 * Math.sqrt(5) / 2 * 12;
                case 5:
                    robot.setMotorPowerPercentage(-.2,.2);
                    desired_yaw = placeholderForNow;
                case 6:
                    robot.setMotorPowerPercentage(default_speed,default_speed);
                    desired_distance = 5 * 12;
            }

        }

    }
    public boolean cellThere(){
        return Math.random() < 0.5;
    }

    public void collectBall(){
        //do something
    }
}
