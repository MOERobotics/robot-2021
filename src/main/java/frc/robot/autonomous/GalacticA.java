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
    double start_inches = 0;
    double startingYaw = 0;
    double yawChange = 0;
    int autonomousStep = 0;
    double correction = 0;


    @Override
    protected void printSmartDashboardInternal(){
        SmartDashboard.putNumber("Autonomous Step", autonomousStep);

        SmartDashboard.putBoolean("red", red);
        SmartDashboard.putBoolean("blue", blue);
        SmartDashboard.putBoolean("cell check", cell_check);

        SmartDashboard.putNumber("Desired Yaw", desired_yaw);
        SmartDashboard.putNumber("Starting Yaw", startingYaw);
        SmartDashboard.putNumber("Yaw Change", yawChange);

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
            inches_traveled = (robot.getDistanceInchesLeft() - start_inches); // subtract start distance to get distance traveled

            if (inches_traveled >= desired_distance) { // drive up to power cell at C3
                robot.setMotorPowerPercentage(0,0);
                // check if power cell is there
                start_inches = robot.getDistanceInchesLeft();
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


            switch (autonomousStep){
                case 0: // turn at C3
                    robot.setMotorPowerPercentage(-.2,.2);
                    yawChange = robot.getYaw() - startingYaw;
                    if (yawChange < 0) {
                        yawChange += 360;
                    }
                    yawChange = Math.min(yawChange, 360 - yawChange);
                    if (yawChange >= 7.125) {
                        robot.setMotorPowerPercentage(0, 0);
                        start_inches = robot.getDistanceInchesLeft();
                        startingYaw = robot.getYaw();
                        myPID.reset();
                        myPID.enableContinuousInput(-180, 180);
                        autonomousStep += 1;
                    }
                    break;
                case 1: // drive to next power cell
                    correction = myPID.calculate(robot.getYaw() - startingYaw);
                    robot.setMotorPowerPercentage(default_speed*(1+correction),default_speed*(1-correction));
                    inches_traveled = (robot.getDistanceInchesLeft() - start_inches);
                    if (inches_traveled >= 5 * Math.sqrt(5)/ 2 * 12) {
                        robot.setMotorPowerPercentage(0, 0);
                        start_inches = robot.getDistanceInchesLeft();
                        startingYaw = robot.getYaw();
                        myPID.reset();
                        myPID.enableContinuousInput(-180, 180);
                        autonomousStep += 1;
                    }
                    break;
                case 2: // turn
                    robot.setMotorPowerPercentage(-.2,.2);
                    desired_yaw = 98.13;
                    yawChange = robot.getYaw() - startingYaw;
                    if (yawChange < 0) {
                        yawChange += 360;
                    }
                    yawChange = Math.min(yawChange, 360 - yawChange);
                    if (yawChange >= 98.13) {
                        robot.setMotorPowerPercentage(0, 0);
                        start_inches = robot.getDistanceInchesLeft();
                        startingYaw = robot.getYaw();
                        myPID.reset();
                        myPID.enableContinuousInput(-180, 180);
                        autonomousStep += 1;
                    }
                    break;
                case 3: // drive to next power cell
                    correction = myPID.calculate(robot.getYaw() - startingYaw);
                    robot.setMotorPowerPercentage(default_speed*(1+correction),default_speed*(1-correction));
                    desired_distance = 5*Math.sqrt(10)/2*12;
                    inches_traveled = (robot.getDistanceInchesLeft() - start_inches);
                    if (inches_traveled >= 5 * Math.sqrt(10)/ 2 * 12) {
                        robot.setMotorPowerPercentage(0, 0);
                        start_inches = robot.getDistanceInchesLeft();
                        startingYaw = robot.getYaw();
                        myPID.reset();
                        myPID.enableContinuousInput(-180, 180);
                        autonomousStep += 1;
                    }
                    break;
                case 4: //turn
                    robot.setMotorPowerPercentage(.2,-.2);
                    desired_yaw = 71.565;
                    yawChange = robot.getYaw() - startingYaw;
                    if (yawChange < 0) {
                        yawChange += 360;
                    }
                    yawChange = Math.min(yawChange, 360 - yawChange);
                    if (yawChange >= 71.565) {
                        robot.setMotorPowerPercentage(0, 0);
                        start_inches = robot.getDistanceInchesLeft();
                        startingYaw = robot.getYaw();
                        myPID.reset();
                        myPID.enableContinuousInput(-180, 180);
                        autonomousStep += 1;
                    }
                    break;
                case 5: //drive to end zone
                    correction = myPID.calculate(robot.getYaw() - startingYaw);
                    robot.setMotorPowerPercentage(default_speed*(1+correction),default_speed*(1-correction));
                    desired_distance = 25*6;
                    inches_traveled = (robot.getDistanceInchesLeft() - start_inches);
                    if (inches_traveled >= 25*6) {
                        robot.setMotorPowerPercentage(0, 0);
                        start_inches = robot.getDistanceInchesLeft();
                        startingYaw = robot.getYaw();
                        myPID.reset();
                        myPID.enableContinuousInput(-180, 180);
                        autonomousStep += 1;
                    }
                    break;
            }

        }



        if (blue){
            switch (autonomousStep){
                case 0: // drive to power cell from C3
                    correction = myPID.calculate(robot.getYaw() - startingYaw);
                    robot.setMotorPowerPercentage(default_speed*(1+correction),default_speed*(1-correction));
                    inches_traveled = (robot.getDistanceInchesLeft() - start_inches);
                    if (inches_traveled >= 5*Math.sqrt(13)/2*12) {
                        robot.setMotorPowerPercentage(0, 0);
                        start_inches = robot.getDistanceInchesLeft();
                        startingYaw = robot.getYaw();
                        myPID.reset();
                        myPID.enableContinuousInput(-180, 180);
                        autonomousStep += 1;
                    }
                    break;
                case 1: // turn
                    robot.setMotorPowerPercentage(-.2,.2);
                    yawChange = robot.getYaw() - startingYaw;
                    if (yawChange < 0) {
                        yawChange += 360;
                    }
                    yawChange = Math.min(yawChange, 360 - yawChange);
                    if (yawChange >= 105.255) {
                        robot.setMotorPowerPercentage(0, 0);
                        start_inches = robot.getDistanceInchesLeft();
                        startingYaw = robot.getYaw();
                        myPID.reset();
                        myPID.enableContinuousInput(-180, 180);
                        autonomousStep += 1;
                    }
                    break;
                case 2: // drive to power cell
                    correction = myPID.calculate(robot.getYaw() - startingYaw);
                    robot.setMotorPowerPercentage(default_speed*(1+correction),default_speed*(1-correction));
                    desired_distance = 5 * Math.sqrt(10) / 2 * 12;
                    inches_traveled = (robot.getDistanceInchesLeft() - start_inches);
                    if (inches_traveled >= 5 * Math.sqrt(10) / 2 * 12) {
                        robot.setMotorPowerPercentage(0, 0);
                        start_inches = robot.getDistanceInchesLeft();
                        startingYaw = robot.getYaw();
                        myPID.reset();
                        myPID.enableContinuousInput(-180, 180);
                        autonomousStep += 1;
                    }
                    break;
                case 3: //turn
                    robot.setMotorPowerPercentage(.2,-.2);
                    desired_yaw = 98.13;
                    yawChange = robot.getYaw() - startingYaw;
                    if (yawChange < 0) {
                        yawChange += 360;
                    }
                    yawChange = Math.min(yawChange, 360 - yawChange);
                    if (yawChange >= 98.13) {
                        robot.setMotorPowerPercentage(0, 0);
                        start_inches = robot.getDistanceInchesLeft();
                        startingYaw = robot.getYaw();
                        myPID.reset();
                        myPID.enableContinuousInput(-180, 180);
                        autonomousStep += 1;
                    }
                    break;
                case 4: //drive to power cell
                    correction = myPID.calculate(robot.getYaw() - startingYaw);
                    robot.setMotorPowerPercentage(default_speed*(1+correction),default_speed*(1-correction));
                    desired_distance = 5 * Math.sqrt(5) / 2 * 12;
                    inches_traveled = (robot.getDistanceInchesLeft() - start_inches);
                    if (inches_traveled >= 5 * Math.sqrt(5) / 2 * 12) {
                        robot.setMotorPowerPercentage(0, 0);
                        start_inches = robot.getDistanceInchesLeft();
                        startingYaw = robot.getYaw();
                        myPID.reset();
                        myPID.enableContinuousInput(-180, 180);
                        autonomousStep += 1;
                    }
                    break;
                case 5: // turn
                    robot.setMotorPowerPercentage(-.2,.2);
                    desired_yaw = 26.565;
                    yawChange = robot.getYaw() - startingYaw;
                    if (yawChange < 0) {
                        yawChange += 360;
                    }
                    yawChange = Math.min(yawChange, 360 - yawChange);
                    if (yawChange >= 26.565) {
                        robot.setMotorPowerPercentage(0, 0);
                        start_inches = robot.getDistanceInchesLeft();
                        startingYaw = robot.getYaw();
                        myPID.reset();
                        myPID.enableContinuousInput(-180, 180);
                        autonomousStep += 1;
                    }
                    break;
                case 6: // drive to end zone
                    correction = myPID.calculate(robot.getYaw() - startingYaw);
                    robot.setMotorPowerPercentage(default_speed*(1+correction),default_speed*(1-correction));
                    desired_distance = 5 * 12;
                    inches_traveled = (robot.getDistanceInchesLeft() - start_inches);
                    if (inches_traveled >= 5 * 12) {
                        robot.setMotorPowerPercentage(0, 0);
                        start_inches = robot.getDistanceInchesLeft();
                        startingYaw = robot.getYaw();
                        myPID.reset();
                        myPID.enableContinuousInput(-180, 180);
                        autonomousStep += 1;
                    }
                    break;
            }

        }



    }
    public boolean cellThere(){
        return Math.random() < 0.5;
    }

}
