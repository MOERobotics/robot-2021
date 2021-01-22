package frc.robot.autonomous;

import frc.robot.genericrobot.GenericRobot;
import edu.wpi.first.wpilibj.controller.PIDController;

public class BounceA6A9 extends GenericAutonomous{

    double defaultSpeed = .25;
    static double startingYaw = 0;
    double correction;
    static double currentYaw = 0;
    double yawDifference = 0;
    double startingDistance = 0;
    double yawTarget = 0;
    double outerRadius = 0;
    double outerArcLength = 0;
    double innerArcLength = 0;
    double currentDistance = 0;

    @Override
    public void autonomousInit(GenericRobot robot) {
        autonomousStep = -1;
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot){
        PIDController PIDBounce69 = new PIDController(robot.getPIDpivotP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD()) {};
        currentYaw = 0;
        switch(autonomousStep) {
            case -1: //start-up
                PIDBounce69.reset();
                PIDBounce69.disableContinuousInput();
                startingYaw = robot.getYaw();
                startingDistance = robot.getDistanceInchesLeft();
                autonomousStep += 1;
                break;
            case 0: //D5 to A6
                PIDBounce69.enableContinuousInput(-180,180);
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / (180 * Math.PI));
                correction = PIDBounce69.calculate(outerRadius * yawDifference - (robot.getDistanceInchesLeft() - startingDistance));
                robot.setMotorPowerPercentage((-defaultSpeed * 1) * (1 + correction), (-defaultSpeed * 1) * (1 - correction));
                currentDistance = robot.getDistanceInchesRight();
                if(currentDistance - startingDistance > outerArcLength){
                    autonomousStep += 1;
                }
                break;
            case 1: //PID reset for first straight away path
                startingDistance = robot.getDistanceInchesLeft();
                PIDBounce69.reset();
                PIDBounce69.enableContinuousInput(-180,180);
                currentYaw = 0;
                autonomousStep += 1;
                break;
            case 2: //straight away #1
                correction = PIDBounce69.calculate(robot.getYaw() - currentYaw);
                robot.setMotorPowerPercentage((-defaultSpeed * 1) * (1 + correction), (-defaultSpeed * 1) * (1 - correction));
                currentDistance = robot.getDistanceInchesLeft();
                if(currentDistance - startingDistance > 0) { //straight away distance needed to travel not known
                    robot.driveForward(0);
                    autonomousStep += 1;
                }
                break;
            case 3: //PID reset for A6 bounce
                startingDistance = robot.getDistanceInchesLeft();
                PIDBounce69.reset();
                PIDBounce69.enableContinuousInput(-180,180);
                currentYaw = 0;
                autonomousStep += 1;
                break;
            case 4: //A6 bounce
                correction = PIDBounce69.calculate(robot.getYaw() - currentYaw);
                robot.setMotorPowerPercentage((-defaultSpeed * 1) * (1 + correction), (defaultSpeed * 1) * (1 - correction));
                currentDistance = robot.getDistanceInchesLeft();
                if(currentDistance - startingDistance > 0) { //distnace not known
                    robot.driveForward(0);
                    autonomousStep += 1;
                }
                break;
            case 5: //PID reset for Second straight away
                startingDistance = robot.getDistanceInchesLeft();
                PIDBounce69.reset();
                PIDBounce69.enableContinuousInput(-180,180);
                currentYaw = 0;
                autonomousStep += 1;
                break;
            case 6: //straight away #2
                correction = PIDBounce69.calculate(robot.getYaw() - currentYaw);
                robot.setMotorPowerPercentage((defaultSpeed * 1) * (1 + correction), (defaultSpeed * 1) * (1 - correction));
                currentDistance = robot.getDistanceInchesLeft();
                if(currentDistance - startingDistance > 0) { //straight away distance needed to travel not known
                    robot.driveForward(0);
                    autonomousStep += 1;
                }
                break;
            case 7: //PID reset for Second straight away
                startingDistance = robot.getDistanceInchesLeft();
                PIDBounce69.reset();
                PIDBounce69.enableContinuousInput(-180,180);
                currentYaw = 0;
                autonomousStep += 1;
                break;
            case 8: //D5 to A6
                PIDBounce69.enableContinuousInput(-180,180);
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / (180 * Math.PI));
                correction = PIDBounce69.calculate(outerRadius * yawDifference - (robot.getDistanceInchesLeft() - startingDistance));
                robot.setMotorPowerPercentage((defaultSpeed * 1) * (1 + correction), (defaultSpeed * 1) * (1 - correction));
                currentDistance = robot.getDistanceInchesRight();
                if(currentDistance - startingDistance > innerArcLength){
                    autonomousStep += 1;
                }
                break;
        }
    }

}
