package frc.robot.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.genericrobot.GenericRobot;

public class FullBounce extends GenericAutonomous{

    double defaultSpeed = 0.15;
    static double startingYaw = 0.0;
    double correction;
    static double currentYaw = 0.0;
    double yawDifference = 0.0;
    double startingDistance = 0.0;
    double outerRadius = 0.0;
    double outerArcLength = 0.0;
    double currentDistance = 0.0;
    long startingTime;
    double deltaDistance;
    double smartSpeedCoeff;
    PIDController PIDBounce;

    @Override
    public void autonomousInit(GenericRobot robot) {
        autonomousStep = -1;
        startingTime = 0;
        PIDBounce = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
        currentYaw = 0;
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot){
        switch(autonomousStep) {
            case -1: //start-up
                PIDBounce.reset();
                PIDBounce.disableContinuousInput();
                robot.resetEncoders();
                robot.resetAttitude();
                startingYaw = 0;
                startingDistance = robot.getDistanceInchesRight();
                outerRadius = 37;
                outerArcLength = (2 * Math.PI * outerRadius)/4; // Needs to be tested
                if (System.currentTimeMillis() >= startingTime + 100) {
                    autonomousStep += 1;
                }
                break;
            case 0: //90 degree arc to A3
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / (180 * Math.PI));
                currentDistance = robot.getDistanceInchesRight();
                correction = PIDBounce.calculate(outerRadius * yawDifference + (currentDistance - startingDistance));
                robot.setMotorPowerPercentage((defaultSpeed * 0.75) * (1 + correction), (defaultSpeed * 1.5) * (1 - correction));
                if(currentDistance - startingDistance > outerArcLength){
                    autonomousStep += 1;
                }
                break;
            case 1: //PID reset for straight away one
                startingDistance = robot.getDistanceInchesLeft();
                PIDBounce.reset();
                currentYaw = -90; // Needs to be tested
                PIDBounce.enableContinuousInput(-180, 180);
                autonomousStep += 1;
                break;
            case 2: //straight away one
                correction = PIDBounce.calculate(robot.getYaw() - currentYaw);
                robot.setMotorPowerPercentage(defaultSpeed * (1 + correction), defaultSpeed * (1 - correction));
                currentDistance = robot.getDistanceInchesLeft();
                if (currentDistance - startingDistance > 48) { // Needs to travel 2 feet (24)
                    robot.driveForward(0);
                    autonomousStep += 1;
                }
                break;
            case 3: //PID reset for bounce
                startingDistance = robot.getDistanceInchesLeft();
                PIDBounce.reset();
                outerRadius = 26;
                outerArcLength = (Math.PI * outerRadius)/6; //Needs to be tested
                PIDBounce.disableContinuousInput();
                autonomousStep += 1;
                break;
            case 4: //Bounce backwards right arc
                correction = PIDBounce.calculate(outerRadius * yawDifference + (robot.getDistanceInchesLeft() - startingDistance));
                robot.setMotorPowerPercentage(-(defaultSpeed * 1.5) * (1 + correction), -(defaultSpeed * 0.75) * (1 - correction));
                currentDistance = robot.getDistanceInchesLeft();
                if (currentDistance - startingDistance < -outerArcLength) { //Test
                    currentYaw = robot.getYaw();
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
                if (currentDistance - startingDistance < -60) { // Needs to travel ~2.5 feet (30)
                    robot.driveForward(0);
                    autonomousStep += 1;
                }
                break;
            case 7: //PID reset for 150 degree arc to A6
                startingDistance = robot.getDistanceInchesLeft();
                PIDBounce.reset();
                PIDBounce.disableContinuousInput();
                outerRadius = 46; //Test
                outerArcLength = (5 * Math.PI * outerRadius)/6; // Needs to be tested
                autonomousStep = 25;
                break;
            case 8: //150 degree arc to A6
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
                if (currentDistance - startingDistance < -108) { // Needs to travel ~4.5 feet (54)
                    robot.driveForward(0);
                    autonomousStep += 1;
                }
            case 11: //PID reset for A6 bounce
                startingDistance = robot.getDistanceInchesLeft();
                PIDBounce.reset();
                PIDBounce.disableContinuousInput();
                currentYaw = 0;
                outerRadius = 26;
                outerArcLength = (Math.PI * outerRadius)/6; //Needs to be tested
                autonomousStep += 1;
                break;
            case 12: //A6 bounce
                correction = PIDBounce.calculate(outerRadius * yawDifference + (robot.getDistanceInchesLeft() - startingDistance));
                robot.setMotorPowerPercentage((defaultSpeed * 1) * (1 - correction), (defaultSpeed * 1) * (1 + correction));
                currentDistance = robot.getDistanceInchesLeft();
                if(currentDistance - startingDistance > outerArcLength) {
                    robot.driveForward(0);
                    autonomousStep += 1;
                }
                break;
            case 13: //PID reset for Second straight away
                startingDistance = robot.getDistanceInchesLeft();
                PIDBounce.reset();
                PIDBounce.enableContinuousInput(-180,180);
                currentYaw = 0;
                autonomousStep += 1;
                break;
            case 14: //straight away #2
                correction = PIDBounce.calculate(robot.getYaw() - currentYaw);
                robot.setMotorPowerPercentage((defaultSpeed * 1) * (1 - correction), (defaultSpeed * 1) * (1 + correction));
                currentDistance = robot.getDistanceInchesLeft();
                if(currentDistance - startingDistance > 24) { //Needs to be tested (12)
                    robot.driveForward(0);
                    autonomousStep += 1;
                }
                break;
            case 15: //PID reset for D7/D8 arc (D7 portion)
                startingDistance = robot.getDistanceInchesLeft();
                PIDBounce.reset();
                PIDBounce.disableContinuousInput();
                currentYaw = 0;
                outerRadius = 24;
                outerArcLength = (2 * Math.PI * outerRadius)/4; //Needs to be tested
                autonomousStep += 1;
                break;
            case 16: //D7 arc
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / (180 * Math.PI));
                correction = PIDBounce.calculate(outerRadius * yawDifference + (robot.getDistanceInchesLeft() - startingDistance));
                robot.setMotorPowerPercentage((defaultSpeed * 0.75) * (1 + correction), (defaultSpeed * 1.5) * (1 - correction));
                currentDistance = robot.getDistanceInchesRight();
                if(currentDistance - startingDistance > outerArcLength){
                    autonomousStep += 1;
                }
                break;
            case 17: //PID reset for Third straight away
                startingDistance = robot.getDistanceInchesLeft();
                PIDBounce.reset();
                PIDBounce.enableContinuousInput(-180,180);
                currentYaw = 0;
                autonomousStep += 1;
                break;
            case 18: //straight away #3
                correction = PIDBounce.calculate(robot.getYaw() - currentYaw);
                robot.setMotorPowerPercentage((defaultSpeed * 1) * (1 - correction), (defaultSpeed * 1) * (1 + correction));
                currentDistance = robot.getDistanceInchesLeft();
                if(currentDistance - startingDistance > 36) { //Needs to be tested (36)
                    robot.driveForward(0);
                    autonomousStep += 1;
                }
                break;
            case 19: //PID reset for D7/D8 arc (D8 portion)
                startingDistance = robot.getDistanceInchesLeft();
                PIDBounce.reset();
                PIDBounce.disableContinuousInput();
                currentYaw = 0;
                outerRadius = 24;
                outerArcLength = (4 * Math.PI * outerRadius)/9; //Needs to be tested
                autonomousStep += 1;
                break;
            case 20: //D8 arc
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / (180 * Math.PI));
                correction = PIDBounce.calculate(outerRadius * yawDifference + (robot.getDistanceInchesLeft() - startingDistance));
                robot.setMotorPowerPercentage((defaultSpeed * 0.75) * (1 + correction), (defaultSpeed * 1.5) * (1 - correction));
                currentDistance = robot.getDistanceInchesRight();
                if(currentDistance - startingDistance > outerArcLength){
                    autonomousStep += 1;
                }
                break;
            case 21: //PID reset for Fourth straight away
                startingDistance = robot.getDistanceInchesLeft();
                PIDBounce.reset();
                PIDBounce.enableContinuousInput(-180,180);
                currentYaw = 0;
                autonomousStep += 1;
                break;
            case 22: //straight away #4
                correction = PIDBounce.calculate(robot.getYaw() - currentYaw);
                robot.setMotorPowerPercentage((defaultSpeed * 1) * (1 - correction), (defaultSpeed * 1) * (1 + correction));
                currentDistance = robot.getDistanceInchesLeft();
                if(currentDistance - startingDistance > 185) { //Needs to be tested (92.5)
                    robot.driveForward(0);
                    autonomousStep += 1;
                }
                break;
            case 23: //PID reset for arc to finish zone
                startingDistance = robot.getDistanceInchesLeft();
                PIDBounce.reset();
                PIDBounce.disableContinuousInput();
                currentYaw = 0;
                outerRadius = 36;
                outerArcLength = (11 * Math.PI * outerRadius)/18; //Needs to be tested
                autonomousStep += 1;
                break;
            case 24: //Final arc
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / (180 * Math.PI));
                correction = PIDBounce.calculate(outerRadius * yawDifference + (robot.getDistanceInchesLeft() - startingDistance));
                robot.setMotorPowerPercentage((-defaultSpeed * 1.5) * (1 + correction), (-defaultSpeed * 0.75) * (1 - correction));
                currentDistance = robot.getDistanceInchesRight();
                if(currentDistance - startingDistance < -outerArcLength){
                    autonomousStep += 1;
                }
                break;
            case 25:
                robot.setMotorPowerPercentage(0, 0);
                break;
        }
    }

    //Methods for common autonomous routine maneuvers
    public void autoInitiate(GenericRobot robot) {
        PIDBounce.reset();
        PIDBounce.disableContinuousInput();
        robot.resetAttitude();
        robot.resetEncoders();
        startingDistance = robot.getDistanceInchesRight();

        currentYaw = 0;
        if (System.currentTimeMillis() >= startingTime + 100)
            autonomousStep += 1;
    }

    public void arcReset(double getDistLeft, double getYaw) {
        PIDBounce.reset();
        PIDBounce.disableContinuousInput();
        startingDistance = getDistLeft;
        startingYaw = getYaw;

        autonomousStep += 1;
    }

    public void straightReset(double optimalYaw, double getDistLeft, double getYaw) {
        PIDBounce.reset();
        PIDBounce.enableContinuousInput(-180, 180);

        currentYaw = optimalYaw; //0 if facing forward, 180 if facing backward (relative to starting position)
        startingDistance = getDistLeft;
        startingYaw = getYaw;

        autonomousStep += 1;
    }


    public void straightaway(GenericRobot robot, double getYaw, double getDistLeft, double finalDistance) {
        correction = PIDBounce.calculate(getYaw - currentYaw);
        robot.setMotorPowerPercentage(defaultSpeed * (1 + correction), defaultSpeed * (1 - correction));

        currentDistance = getDistLeft;

        if (currentDistance - startingDistance > finalDistance)
            autonomousStep += 1;
    }

    public void arcLeft(GenericRobot robot, double getYaw, double getDistRight, double outArcLength) {
        yawDifference = continuousAngleDiff((getYaw - startingYaw) / 180 * Math.PI);
        currentDistance = getDistRight;
        correction = PIDBounce.calculate((currentDistance - startingDistance) + outerRadius * yawDifference);
        robot.setMotorPowerPercentage((defaultSpeed * .75) * (1 + correction), (defaultSpeed * 1.5) * (1 - correction));

        if (currentDistance - startingDistance > outArcLength)
            autonomousStep += 1;
    }


    public void arcRight(GenericRobot robot, double getYaw, double getDistLeft, double outArcLength) {
        yawDifference = continuousAngleDiff((getYaw - startingYaw) / 180 * Math.PI);
        currentDistance = getDistLeft;
        correction = PIDBounce.calculate(-(currentDistance - startingDistance) + outerRadius * yawDifference);
        robot.setMotorPowerPercentage((defaultSpeed * 1.5) * (1 + correction), (defaultSpeed * .75) * (1 - correction));

        if (currentDistance - startingDistance > outArcLength)
            autonomousStep += 1;
    }


    public double rightArcDiff(double deltaTheta) {
        if (deltaTheta < 360) {
            deltaTheta += 360;
        }
        deltaTheta = (deltaTheta * Math.PI) / 180;

        return deltaTheta;
    }

    public double getSmartSpeedCoeff(double startingDistance, double currentDistance, double finalDistance) { //implement acceleration & deceleration (not yet implemented)

        deltaDistance = currentDistance - startingDistance;
        /*

        (% of default speed)  33   | 66    | 100   | 66    | 33
        (% of final distance) 0-20 | 20-35 | 35-80 | 80-90 | 90-100

         */

        if (deltaDistance < (finalDistance * .2)) { //0-20
            smartSpeedCoeff = 0.33;
        }

        if (deltaDistance >= (finalDistance * .2) && (deltaDistance < (finalDistance * .35))) { //20-35
            smartSpeedCoeff = 0.66;
        }

        if (deltaDistance >= (finalDistance * .35) && (deltaDistance < (finalDistance * .8))) { //35-80
            smartSpeedCoeff = 1.00;
        }

        if (deltaDistance >= (finalDistance * .8) && (deltaDistance < (finalDistance * .9))) { //80-90
            smartSpeedCoeff = 0.66;
        }

        if (deltaDistance >= (finalDistance * .9) && (deltaDistance < finalDistance)) { //90-100
            smartSpeedCoeff = 0.33;
        }


        return smartSpeedCoeff; //defaultSpeed * smartSpeed(startingDistance, currentDistance), finalDistance)j
    }
}
