package frc.robot.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.GenericRobot;
import java.lang.Math;

// robot starts in 33.7 degree angle to the x-axis, facing SE. Robot breaks plane of A1B1 in start zone\
public class GalacticA extends GenericAutonomous {

    PIDController myPID = new PIDController(0,0,0);

    double inches_traveled = 0;
    double desired_distance = 72;
    double desired_yaw = 0;
    boolean cell_check = true;
    boolean red = false;
    boolean blue = false;
    double default_speed = .2;
    double start_ticks = 0;
    double startingYaw = 0;
    double yawChange = 0;
    int autonomousStep = 0;
    double correction = 0;


    @Override
    protected void printSmartDashboardInternal(){
        SmartDashboard.putNumber("Case", autonomousStep);

        SmartDashboard.putBoolean("red", red);
        SmartDashboard.putBoolean("blue", blue);
        SmartDashboard.putBoolean("cell check", cell_check);

        SmartDashboard.putNumber("desired Yaw", desired_yaw);
        SmartDashboard.putNumber("starting Yaw", startingYaw);
        SmartDashboard.putNumber("Yaw change", yawChange);

        SmartDashboard.putNumber("inches traveled", inches_traveled);
        SmartDashboard.putNumber("start ticks", start_ticks);
        SmartDashboard.putNumber("desired distance", desired_distance);

    }
    @Override
    public void autonomousInit(GenericRobot robot){
        start_ticks = robot.getDistanceInchesLeft();
        startingYaw = robot.getYaw();
        autonomousStep = 0;
        red = false;
        blue = false;
        cell_check = true;
        desired_distance = 20*Math.sqrt(13);
        myPID = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
        myPID.reset();
        myPID.enableContinuousInput(-180,180);

    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {

        if (cell_check){
            correction = myPID.calculate(robot.getYaw() - startingYaw);
            robot.setMotorPowerPercentage(default_speed*(1+correction),default_speed*(1-correction));
            inches_traveled = (robot.getDistanceInchesLeft() - start_ticks); // subtract start distance to get distance traveled

            if (inches_traveled >= desired_distance) { // drive up to power cell at C3
                robot.setMotorPowerPercentage(0,0);
                // check if power cell is there
                start_ticks = robot.getDistanceInchesLeft();
                startingYaw = robot.getYaw();
                myPID.reset();
                myPID.enableContinuousInput(-180,180);

                if (cellThere()){ // check if cell is there
                    red = true;
                    desired_distance = 5*Math.sqrt(5)/2*12;
                    desired_yaw = 7.125;

                }
                else{
                    blue = true;
                    desired_distance = 5*Math.sqrt(13)/2*12;
                    desired_yaw = 105.255;
                }
                cell_check = false;


            }
        }


        if (red){
            if (autonomousStep==1 || autonomousStep==3 || autonomousStep==5) { // only when not turning
                inches_traveled = (robot.getDistanceInchesLeft() - start_ticks);
                if (inches_traveled >= desired_distance) {
                    robot.setMotorPowerPercentage(0, 0);
                    start_ticks = robot.getDistanceInchesLeft();
                    startingYaw = robot.getYaw();
                    autonomousStep += 1;
                }
            }
            if (autonomousStep==0||autonomousStep==2||autonomousStep==4) {
                yawChange = robot.getYaw() - startingYaw;
                if (yawChange < 0) {
                    yawChange += 360;
                }
                yawChange = Math.min(yawChange, 360 - yawChange);
                if (yawChange >= desired_yaw) {
                    robot.setMotorPowerPercentage(0, 0);
                    startingYaw = robot.getYaw();
                    start_ticks = robot.getDistanceInchesLeft();
                    myPID.reset();
                    myPID.enableContinuousInput(-180, 180);
                    autonomousStep += 1;
                }
            }

            switch (autonomousStep){
                case 0: // turn at C3
                    robot.setMotorPowerPercentage(-.2,.2);
                    break;
                case 1: // drive to next power cell
                    correction = myPID.calculate(robot.getYaw() - startingYaw);
                    robot.setMotorPowerPercentage(default_speed*(1+correction),default_speed*(1-correction));
                    break;
                case 2: // turn
                    robot.setMotorPowerPercentage(-.2,.2);
                    desired_yaw = 98.13;
                    break;
                case 3: // drive to next power cell
                    correction = myPID.calculate(robot.getYaw() - startingYaw);
                    robot.setMotorPowerPercentage(default_speed*(1+correction),default_speed*(1-correction));
                    desired_distance = 5*Math.sqrt(10)/2*12;
                    break;
                case 4: //turn
                    robot.setMotorPowerPercentage(.2,-.2);
                    desired_yaw = 71.565;
                    break;
                case 5: //drive to end zone
                    correction = myPID.calculate(robot.getYaw() - startingYaw);
                    robot.setMotorPowerPercentage(default_speed*(1+correction),default_speed*(1-correction));
                    desired_distance = 25*6;
                    break;
            }

        }



        if (blue){
            if (autonomousStep == 0 || autonomousStep == 2|| autonomousStep==4|| autonomousStep==6) {
                inches_traveled = (robot.getDistanceInchesLeft() - start_ticks);
                if (inches_traveled >= desired_distance) {
                    robot.setMotorPowerPercentage(0, 0);
                    start_ticks = robot.getDistanceInchesLeft();
                    startingYaw = robot.getYaw();
                    autonomousStep += 1;
                }
            }
            if (autonomousStep ==1||autonomousStep==3||autonomousStep==5) {
                yawChange = robot.getYaw() - startingYaw;
                if (yawChange < 0) {
                    yawChange += 360;
                }
                yawChange = Math.min(yawChange, 360 - yawChange);
                if (yawChange >= desired_yaw) {
                    robot.setMotorPowerPercentage(0, 0);
                    start_ticks = robot.getDistanceInchesLeft();
                    startingYaw = robot.getYaw();
                    myPID.reset();
                    myPID.enableContinuousInput(-180, 180);
                    autonomousStep += 1;
                }
            }
            switch (autonomousStep){
                case 0: // drive to power cell from C3
                    correction = myPID.calculate(robot.getYaw() - startingYaw);
                    robot.setMotorPowerPercentage(default_speed*(1+correction),default_speed*(1-correction));
                    break;
                case 1: // turn
                    robot.setMotorPowerPercentage(-.2,.2);
                    break;
                case 2: // drive to power cell
                    correction = myPID.calculate(robot.getYaw() - startingYaw);
                    robot.setMotorPowerPercentage(default_speed*(1+correction),default_speed*(1-correction));
                    desired_distance = 5 * Math.sqrt(10) / 2 * 12;
                    break;
                case 3: //turn
                    robot.setMotorPowerPercentage(.2,-.2);
                    desired_yaw = 98.13;
                    break;
                case 4: //drive to power cell
                    correction = myPID.calculate(robot.getYaw() - startingYaw);
                    robot.setMotorPowerPercentage(default_speed*(1+correction),default_speed*(1-correction));
                    desired_distance = 5 * Math.sqrt(5) / 2 * 12;
                    break;
                case 5: // turn
                    robot.setMotorPowerPercentage(-.2,.2);
                    desired_yaw = 26.565;
                    break;
                case 6: // drive to end zone
                    correction = myPID.calculate(robot.getYaw() - startingYaw);
                    robot.setMotorPowerPercentage(default_speed*(1+correction),default_speed*(1-correction));
                    desired_distance = 5 * 12;
                    break;
            }

        }



    }
    public boolean cellThere(){
        return Math.random() < 0.5;
    }

}
