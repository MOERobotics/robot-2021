package frc.robot.autonomous;

import frc.robot.genericrobot.GenericRobot;
import edu.wpi.first.wpilibj.controller.PIDController;

public class BounceA6A9 extends GenericAutonomous{

    double defaultSpeed = .25;
    static double startingYaw = 0;
    double correction;
    static double currentYaw = 0;
    double yawDifference =  0;
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
            case 0: //A6 bounce (28 + 7in arc length)
                correction = PIDBounce69.calculate(outerRadius * yawDifference - (robot.getDistanceInchesLeft() - startingDistance));
                robot.setMotorPowerPercentage((defaultSpeed * .75) * (1 + correction), (defaultSpeed * 1.5) * (1 - correction));
                currentDistance = robot.getDistanceInchesRight();
                if(currentDistance - startingDistance > 35) {
                    robot.driveForward(0);
                    autonomousStep += 1;
                }
                break;
            case 1: //PID reset for straightaway
                startingDistance = robot.getDistanceInchesRight();
                PIDBounce69.reset();
                PIDBounce69.enableContinuousInput(-180,180);
                currentYaw = 80;
                autonomousStep += 1;
                break;
            case 2: //straight away #1 (65 in)
                correction = PIDBounce69.calculate(robot.getYaw() - currentYaw);
                robot.setMotorPowerPercentage((defaultSpeed * 1) * (1 + correction), (defaultSpeed * 1) * (1 - correction));
                currentDistance = robot.getDistanceInchesRight();
                if(currentDistance - startingDistance > 65) {
                    robot.driveForward(0);
                    autonomousStep += 1;
                }
                break;
            case 3: //PID reset for arc
                startingDistance = robot.getDistanceInchesRight();
                PIDBounce69.reset();
                PIDBounce69.enableContinuousInput(-180,180);
                currentYaw = 0;
                autonomousStep += 1;
                break;
            case 4: //First forward left arc (44 in (28 +(30-14))
                PIDBounce69.enableContinuousInput(-180,180);
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / (180 * Math.PI));
                correction = PIDBounce69.calculate(outerRadius * yawDifference - (robot.getDistanceInchesLeft() - startingDistance));
                robot.setMotorPowerPercentage((defaultSpeed * .75) * (1 + correction), (defaultSpeed * 1.5) * (1 - correction));
                currentDistance = robot.getDistanceInchesRight();
                if(currentDistance - startingDistance > 44){
                    autonomousStep += 1;
                }
                break;
            case 5: //PID reset for straightaway #2
                startingDistance = robot.getDistanceInchesRight();
                PIDBounce69.reset();
                PIDBounce69.enableContinuousInput(-180,180);
                currentYaw = 0;
                autonomousStep += 1;
                break;
            case 6: //straight away #2 (30 in)
                correction = PIDBounce69.calculate(robot.getYaw() - currentYaw);
                robot.setMotorPowerPercentage((defaultSpeed * 1) * (1 + correction), (defaultSpeed * 1) * (1 - correction));
                currentDistance = robot.getDistanceInchesRight();
                if(currentDistance - startingDistance > 30) {
                    robot.driveForward(0);
                    autonomousStep += 1;
                }
                break;
            case 7: //PID reset for arc #2
                startingDistance = robot.getDistanceInchesRight();
                PIDBounce69.reset();
                PIDBounce69.enableContinuousInput(-180,180);
                currentYaw = 280;
                autonomousStep += 1;
                break;
            case 8: //Second forward left arc (44 inch arc (28 +(30-14)))
                PIDBounce69.enableContinuousInput(-180,180);
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / (180 * Math.PI));
                correction = PIDBounce69.calculate(outerRadius * yawDifference - (robot.getDistanceInchesRight() - startingDistance));
                robot.setMotorPowerPercentage((defaultSpeed * .75) * (1 + correction), (defaultSpeed * 1.5) * (1 - correction));
                currentDistance = robot.getDistanceInchesRight();
                if(currentDistance - startingDistance > 44){
                    autonomousStep += 1;
                }
                break;
            case 9: //PID reset for 3rd straightaway
                startingDistance = robot.getDistanceInchesRight();
                PIDBounce69.reset();
                PIDBounce69.enableContinuousInput(-180,180);
                currentYaw = 280;
                autonomousStep += 1;
                break;
            case 10: //straight away #3
                correction = PIDBounce69.calculate(robot.getYaw() - currentYaw);
                robot.setMotorPowerPercentage((defaultSpeed * 1) * (1 + correction), (defaultSpeed * 1) * (1 - correction));
                currentDistance = robot.getDistanceInchesRight();
                if(currentDistance - startingDistance > 78) { //straight away distance needed to travel not known
                    robot.driveForward(0);
                    autonomousStep += 1;
                }
                break;
            case 11: //PID reset for A9 Bounce
                startingDistance = robot.getDistanceInchesRight();
                PIDBounce69.reset();
                PIDBounce69.enableContinuousInput(-180,180);
                currentYaw = 260;
                autonomousStep += 1;
                break;
            case 12: //A9 bounce
                correction = PIDBounce69.calculate(outerRadius * yawDifference - (robot.getDistanceInchesRight() - startingDistance));
                robot.setMotorPowerPercentage(-(defaultSpeed * 1.5) * (1 + correction), -(defaultSpeed * .75) * (1 - correction));
                currentDistance = robot.getDistanceInchesRight();
                if(currentDistance - startingDistance > 35) { //distance not known
                    robot.driveForward(0);
                    autonomousStep += 1;
                }
                break;
            case 13: //PID reset for ending arc
                startingDistance = robot.getDistanceInchesLeft();
                PIDBounce69.reset();
                PIDBounce69.enableContinuousInput(-180,180);
                currentYaw = 180;
                autonomousStep += 1;
                break;
            case 14: //arc to finish zone (44 in (28 +(30-14))
                correction = PIDBounce69.calculate(outerRadius * yawDifference - (robot.getDistanceInchesLeft() - startingDistance));
                robot.setMotorPowerPercentage(-(defaultSpeed * 1.5) * (1 + correction), -(defaultSpeed * .75) * (1 - correction));
                currentDistance = robot.getDistanceInchesLeft();
                if(currentDistance - startingDistance > -44) {
                    robot.driveForward(0);
                    autonomousStep += 1;
                }
                break;
        }
    }

}
