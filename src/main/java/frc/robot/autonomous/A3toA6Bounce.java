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
    double yawTarget = 0.0; //unknown
    double outerRadius = 0.0; //unknown
    double outerArcLength = 0.0; //unknown measurement
    double innerArcLength = 0.0; //unknown measurement
    double currentDistance = 0.0;

    @Override
    public void autonomousInit(GenericRobot robot) {
        autonomousStep = -1;
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot){
        PIDController PIDBounce = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
        currentYaw = 0;
        switch(autonomousStep) {
            case -1: //start-up
                PIDBounce.reset();
                PIDBounce.disableContinuousInput();
                startingYaw = robot.getYaw();
                startingDistance = robot.getDistanceInchesLeft();
                autonomousStep += 1;
                break;
            case 0: //30 degree arc from A3
                PIDBounce.enableContinuousInput(-180, 180);
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / (180 * Math.PI));
                correction = PIDBounce.calculate(outerRadius * yawDifference - (robot.getDistanceInchesLeft() - startingDistance));
                robot.setMotorPowerPercentage((defaultSpeed * 1) * (1 + correction), (defaultSpeed * 1) * (1 - correction));
                currentDistance = robot.getDistanceInchesRight();
                if(currentDistance - startingDistance > outerArcLength){
                    autonomousStep += 1;
                }
                break;
            case 1: //PID reset for straight away one
                startingDistance = robot.getDistanceInchesLeft();
                PIDBounce.reset();
                PIDBounce.enableContinuousInput(-180, 180);
                currentYaw = 0;
                autonomousStep += 1;
                break;
            case 2: //straight away one
                correction = PIDBounce.calculate(robot.getYaw() - currentYaw);
                robot.setMotorPowerPercentage(defaultSpeed * (1 + correction), defaultSpeed * (1 - correction));
                currentDistance = robot.getDistanceInchesLeft();
                if (currentDistance - startingDistance > 0) { //distance unknown
                    robot.driveForward(0);
                    autonomousStep += 1;
                }
                break;
            case 3: //PID reset for bounce
                startingDistance = robot.getDistanceInchesLeft();
                PIDBounce.reset();
                PIDBounce.enableContinuousInput(-180, 180);
                currentYaw = 0;
                autonomousStep += 1;
                break;
            case 4: //bounce
                correction = PIDBounce.calculate(robot.getYaw() - currentYaw);
                robot.setMotorPowerPercentage(-defaultSpeed * (1 + correction), defaultSpeed * (1 - correction));
                currentDistance = robot.getDistanceInchesLeft();
                if (currentDistance - startingDistance > 0) { //distance unknown
                    robot.driveForward(0);
                    autonomousStep += 1;
                }
                break;
            case 5: //PID reset for straight away two
                startingDistance = robot.getDistanceInchesLeft();
                PIDBounce.reset();
                PIDBounce.enableContinuousInput(-180, 180);
                currentYaw = 0;
                autonomousStep += 1;
                break;
            case 6: //straight away two
                correction = PIDBounce.calculate(robot.getYaw() - currentYaw);
                robot.setMotorPowerPercentage(-defaultSpeed * (1 + correction), -defaultSpeed * (1 - correction));
                currentDistance = robot.getDistanceInchesLeft();
                if (currentDistance - startingDistance > 0) { //distance unknown
                    robot.driveForward(0);
                    autonomousStep += 1;
                }
                break;
            case 7: //PID reset for 30 degree arc to A6
                startingDistance = robot.getDistanceInchesLeft();
                PIDBounce.reset();
                PIDBounce.enableContinuousInput(-180, 180);
                currentYaw = 0;
                autonomousStep += 1;
                break;
            case 8: //30 degree arc to A6
                PIDBounce.enableContinuousInput(-180, 180);
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / (180 * Math.PI));
                correction = PIDBounce.calculate(outerRadius * yawDifference - (robot.getDistanceInchesLeft() - startingDistance));
                robot.setMotorPowerPercentage((-defaultSpeed * 1) * (1 + correction), (-defaultSpeed * 1) * (1 - correction));
                currentDistance = robot.getDistanceInchesLeft();
                if(currentDistance - startingDistance > innerArcLength){
                    autonomousStep += 1;
                }
                break;
        }
    }
}
