package frc.robot.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.genericrobot.GenericRobot;

public class A3toA6Bounce extends GenericAutonomous{

    double defaultSpeed = 0.25;
    static double startingYaw = 0.0;
    double correction;
    static double currentYaw = 0.0;
    double yawDifference = 0.0;
    double startingDistance = 0.0;
    double outerRadius = 0.0;
    double outerArcLength = 0.0;
    double currentDistance = 0.0;
    long startingTime;
    PIDController PIDBounce;

    @Override
    public void autonomousInit(GenericRobot robot) {
        autonomousStep = -1;
        PIDBounce = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot){
        currentYaw = 0;
        switch(autonomousStep) {
            case -1: //start-up
                PIDBounce.reset();
                PIDBounce.disableContinuousInput();
                startingYaw = 0;
                startingDistance = robot.getDistanceInchesLeft();
                outerArcLength = 41.25; // Needs to be tested
                if (System.currentTimeMillis() >= startingTime + 100) {
                    autonomousStep += 1;
                }
                break;
            case 0: //90 degree arc from A3
                PIDBounce.enableContinuousInput(-180, 180);
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / (180 * Math.PI));
                correction = PIDBounce.calculate(outerRadius * yawDifference + (robot.getDistanceInchesLeft() - startingDistance));
                robot.setMotorPowerPercentage((defaultSpeed * 0.75) * (1 + correction), (defaultSpeed * 1.5) * (1 - correction));
                currentDistance = robot.getDistanceInchesRight();
                if(currentDistance - startingDistance > outerArcLength){
                    autonomousStep += 1;
                }
                break;
            case 1: //PID reset for straight away one
                startingDistance = robot.getDistanceInchesLeft();
                PIDBounce.reset();
                PIDBounce.enableContinuousInput(-180, 180);
                currentYaw = 270; // Needs to be tested
                autonomousStep += 1;
                break;
            case 2: //straight away one
                correction = PIDBounce.calculate(robot.getYaw() - currentYaw);
                robot.setMotorPowerPercentage(defaultSpeed * (1 + correction), defaultSpeed * (1 - correction));
                currentDistance = robot.getDistanceInchesLeft();
                if (currentDistance - startingDistance > 24) { // Needs to travel 2 feet
                    robot.driveForward(0);
                    autonomousStep += 1;
                }
                break;
            case 3: //PID reset for bounce
                startingDistance = robot.getDistanceInchesLeft();
                PIDBounce.reset();
                PIDBounce.enableContinuousInput(-180, 180);
                autonomousStep += 1;
                break;
            case 4: //Bounce backwards right arc
                correction = PIDBounce.calculate(outerRadius * yawDifference + (robot.getDistanceInchesLeft() - startingDistance));
                robot.setMotorPowerPercentage(-(defaultSpeed * 1.5) * (1 + correction), -(defaultSpeed * 0.75) * (1 - correction));
                currentDistance = robot.getDistanceInchesLeft();
                if (currentDistance - startingDistance > -11.25) { //Test
                    robot.driveForward(0);
                    autonomousStep += 1;
                }
                break;
            case 5: //PID reset for straight away two
                startingDistance = robot.getDistanceInchesLeft();
                PIDBounce.reset();
                PIDBounce.enableContinuousInput(-180, 180);
                autonomousStep += 1;
                break;
            case 6: //straight away two
                correction = PIDBounce.calculate(robot.getYaw() - currentYaw);
                robot.setMotorPowerPercentage(-defaultSpeed * (1 + correction), -defaultSpeed * (1 - correction));
                currentDistance = robot.getDistanceInchesLeft();
                if (currentDistance - startingDistance > -30) { // Needs to travel ~2.5 feet
                    robot.driveForward(0);
                    autonomousStep += 1;
                }
                break;
            case 7: //PID reset for 150 degree arc to A6
                startingDistance = robot.getDistanceInchesLeft();
                PIDBounce.reset();
                PIDBounce.enableContinuousInput(-180, 180);
                outerArcLength = 68.75; // Needs to be tested
                autonomousStep += 1;
                break;
            case 8: //150 degree arc to A6
                PIDBounce.enableContinuousInput(-180, 180);
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / (180 * Math.PI));
                correction = PIDBounce.calculate(outerRadius * yawDifference - (robot.getDistanceInchesLeft() - startingDistance));
                robot.setMotorPowerPercentage((-defaultSpeed * 1.5) * (1 + correction), (-defaultSpeed * 0.75) * (1 - correction));
                currentDistance = robot.getDistanceInchesLeft();
                if(currentDistance - startingDistance > -outerArcLength){
                    autonomousStep += 1;
                }
                break;
            case 9: //PID reset for straight away to A6
                startingDistance = robot.getDistanceInchesLeft();
                PIDBounce.reset();
                PIDBounce.enableContinuousInput(-180, 180);
                autonomousStep += 1;
                break;
            case 10: //straight away to A6
                correction = PIDBounce.calculate(robot.getYaw() - currentYaw);
                robot.setMotorPowerPercentage(-defaultSpeed * (1 + correction), -defaultSpeed * (1 - correction));
                currentDistance = robot.getDistanceInchesLeft();
                if (currentDistance - startingDistance > -54) { // Needs to travel ~4.5 feet
                    robot.driveForward(0);
                    autonomousStep += 1;
                }
        }
    }
}
