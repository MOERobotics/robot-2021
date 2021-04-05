package frc.robot.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.genericrobot.GenericRobot;

public class FullBounce extends GenericAutonomous{

    double defaultSpeed = 0.2;
    static double startingYaw = 0.0;
    double correction;
    static double currentYaw = 0.0;
    double yawDifference = 0.0;
    double startingDistance = 0.0;
    double outerRadius = 0.0;
    double outerArcLength = 0.0;
    double currentDistance = 0.0;
    double straightSpeedScaling = 1.0;
    long startingTime;
    double deltaDistance;
    double smartSpeedCoeff;
    PIDController PIDBounce;
    double innerPowerScaling;
    double outerPowerScaling;

    @Override
    public void autonomousInit(GenericRobot robot) {
        autonomousStep = -3;
        startingTime = System.currentTimeMillis();
        PIDBounce = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
        currentYaw = 0;
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot){
        outerRadius = 44;
        outerPowerScaling = Math.sqrt(outerRadius/(outerRadius-28));
        innerPowerScaling = 1/ outerPowerScaling;
        switch(autonomousStep) {

            case -3: //start up
                robot.resetEncoders();
                robot.resetAttitude();
                startingYaw = 0;
                startingDistance = robot.getDistanceInchesRight();
                PIDBounce.reset();
                currentYaw = 0; // Needs to be tested
                PIDBounce.enableContinuousInput(-180, 180);
                if (System.currentTimeMillis() >= startingTime + 100) {
                    autonomousStep += 1;
                }
                break;

            case -2: //straight away one
                correction = PIDBounce.calculate(robot.getYaw() - currentYaw);
                robot.setMotorPowerPercentage(straightSpeedScaling*defaultSpeed * (1 + correction), straightSpeedScaling*defaultSpeed * (1 - correction));
                currentDistance = robot.getDistanceInchesRight();
                if (currentDistance - startingDistance > 22) { // Needs to travel 2 feet (24)
                    autonomousStep = +1;
                }
                break;
            case -1: //first turn
                PIDBounce.reset();
                PIDBounce.disableContinuousInput();
                startingYaw = 0;
                startingDistance = robot.getDistanceInchesRight();
                outerRadius = 44;
                outerPowerScaling = Math.sqrt(outerRadius/(outerRadius-28));
                innerPowerScaling = 1/ outerPowerScaling;
                outerArcLength = (2 * Math.PI * outerRadius)/4; // Needs to be tested
                break;
            case 0: //90 degree arc to A3
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / (180 * Math.PI));
                currentDistance = robot.getDistanceInchesRight();
                correction = PIDBounce.calculate(outerRadius * yawDifference + (currentDistance - startingDistance));
                robot.setMotorPowerPercentage(innerPowerScaling *defaultSpeed * (1 + correction), outerPowerScaling *defaultSpeed * (1 - correction));
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
                robot.setMotorPowerPercentage(straightSpeedScaling*defaultSpeed * (1 + correction), straightSpeedScaling*defaultSpeed * (1 - correction));
                currentDistance = robot.getDistanceInchesLeft();
                if (currentDistance - startingDistance > 36) { // Needs to travel 2 feet (24)
                    autonomousStep += +1;
                }
                break;
            case 3: //PID reset for small roll backwards
                startingDistance = robot.getDistanceInchesLeft();
                startingYaw = -90;
                PIDBounce.reset();
                PIDBounce.enableContinuousInput(-180, 180);
                autonomousStep += 1;
                break;
            case 4: //small roll backwards
                correction = PIDBounce.calculate(robot.getYaw() - startingYaw);
                robot.setMotorPowerPercentage(-defaultSpeed * (1 - correction), -defaultSpeed * (1 + correction));
                currentDistance = robot.getDistanceInchesLeft();
                if (currentDistance - startingDistance < -18) {
                    autonomousStep += 1;
                }
                break;
            case 5: //PID reset for bounce
                startingDistance = robot.getDistanceInchesLeft();
                startingYaw = -90;
                PIDBounce.reset();
                outerArcLength = (2*Math.PI * outerRadius)/12.0;
                PIDBounce.disableContinuousInput();
                autonomousStep += 1;
                break;
            case 6: //Bounce backwards right arc
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / (180 * Math.PI));
                currentDistance = robot.getDistanceInchesLeft();
                correction = PIDBounce.calculate(outerRadius * yawDifference - (robot.getDistanceInchesLeft() - startingDistance));
                robot.setMotorPowerPercentage(-outerPowerScaling*defaultSpeed * (1 - correction), -innerPowerScaling*defaultSpeed * (1 + correction));
                if (currentDistance - startingDistance < -outerArcLength) {
                    currentYaw = robot.getYaw();
                    autonomousStep += 1;
                }
                break;
            case 7: //PID reset for straight away two
                startingDistance = robot.getDistanceInchesLeft();
                startingYaw = -120;
                PIDBounce.reset();
                PIDBounce.enableContinuousInput(-180, 180);
                autonomousStep += 1;
                break;
            case 8: //straight away two
                correction = PIDBounce.calculate(robot.getYaw() - startingYaw);
                robot.setMotorPowerPercentage(-straightSpeedScaling*defaultSpeed * (1 - correction), -straightSpeedScaling*defaultSpeed * (1 + correction));
                currentDistance = robot.getDistanceInchesLeft();
                if (currentDistance - startingDistance < -64) { //6 ft
                    autonomousStep += 1;
                }
                break;
            case 9: //PID reset for bounce to A6
                startingDistance = robot.getDistanceInchesLeft();
                startingYaw = -120;
                PIDBounce.reset();
                outerArcLength = (2*Math.PI * outerRadius)*60/360;
                PIDBounce.disableContinuousInput();
                autonomousStep += 1;
                break;
            case 10: //Bounce backwards right arc to A6 (1/2)
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / (180 * Math.PI));
                currentDistance = robot.getDistanceInchesLeft();
                correction = PIDBounce.calculate(outerRadius * yawDifference - (robot.getDistanceInchesLeft() - startingDistance));
                robot.setMotorPowerPercentage(-outerPowerScaling*defaultSpeed * (1 - correction), -innerPowerScaling*defaultSpeed * (1 + correction));
                if (currentDistance - startingDistance < -outerArcLength) {
                    currentYaw = robot.getYaw();
                    autonomousStep += 1;
                }
                break;

            case 11:
                straightReset(-180, robot.getDistanceInchesLeft(), robot.getYaw());
                break;

            case 12: //backwards straight
                correction = PIDBounce.calculate(robot.getYaw() - startingYaw);
                robot.setMotorPowerPercentage(-straightSpeedScaling*defaultSpeed * (1 - correction), -straightSpeedScaling*defaultSpeed * (1 + correction));
                currentDistance = robot.getDistanceInchesLeft();
                if (currentDistance - startingDistance < -6) { //6 ft
                    autonomousStep += 1;
                }
                break;

            case 13: //PID reset for bounce to A6
                startingDistance = robot.getDistanceInchesLeft();
                startingYaw = 180;
                PIDBounce.reset();
                outerArcLength = (2*Math.PI * outerRadius)*30/360;
                PIDBounce.disableContinuousInput();
                autonomousStep += 1;
                break;
            case 14: //Bounce backwards right arc to A6 (2/2)
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / (180 * Math.PI));
                currentDistance = robot.getDistanceInchesLeft();
                correction = PIDBounce.calculate(outerRadius * yawDifference - (robot.getDistanceInchesLeft() - startingDistance));
                robot.setMotorPowerPercentage(-outerPowerScaling*defaultSpeed * (1 - correction), -innerPowerScaling*defaultSpeed * (1 + correction));
                if (currentDistance - startingDistance < -outerArcLength) {
                    currentYaw = robot.getYaw();
                    autonomousStep += 1;
                }
                break;
            case 15: //PID reset for backwards straight away three (to A6)
                startingDistance = robot.getDistanceInchesLeft();
                startingYaw = 90;
                PIDBounce.reset();
                PIDBounce.enableContinuousInput(-180, 180);
                autonomousStep += 1;
                break;
            case 16: //backwards straight away three (to A6)
                correction = PIDBounce.calculate(robot.getYaw() - startingYaw);
                robot.setMotorPowerPercentage(-straightSpeedScaling*defaultSpeed * (1 - correction), -straightSpeedScaling*defaultSpeed * (1 + correction));
                currentDistance = robot.getDistanceInchesLeft();
                if (currentDistance - startingDistance < -128) { //6 ft
                    autonomousStep += 1;
                }
                break;
            case 17: //PID reset for forwards roll four (from A6)
                startingDistance = robot.getDistanceInchesLeft();
                startingYaw = 90;
                PIDBounce.reset();
                PIDBounce.enableContinuousInput(-180, 180);
                autonomousStep += 1;
                break;
            case 18: //forwards roll four (from A6)
                correction = PIDBounce.calculate(robot.getYaw() - startingYaw);
                robot.setMotorPowerPercentage(straightSpeedScaling*defaultSpeed * (1 + correction), straightSpeedScaling*defaultSpeed * (1 - correction));
                currentDistance = robot.getDistanceInchesLeft();
                if (currentDistance - startingDistance > 54) { //6 ft > needs testing
                    autonomousStep += 1;
                }
                break;
            case 19: //PID reset for bounce to A9
                startingDistance = robot.getDistanceInchesRight();
                startingYaw = 90;
                PIDBounce.reset();
                //outerRadius = 44; //calculated, needs testing
                outerArcLength = (Math.PI * outerRadius) / 2;
                PIDBounce.disableContinuousInput();
                autonomousStep += 1;
                break;
            case 20: //Bounce left arc to A9
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / (180 * Math.PI));
                currentDistance = robot.getDistanceInchesRight();
                correction = PIDBounce.calculate(outerRadius * yawDifference + (robot.getDistanceInchesRight() - startingDistance));
                robot.setMotorPowerPercentage(defaultSpeed * (1 + correction), defaultSpeed * (1 - correction));
                if (currentDistance - startingDistance > outerArcLength) {
                    currentYaw = robot.getYaw();
                    autonomousStep += 1;
                }
                break;

            case 21:
                 straightReset(0, robot.getDistanceInchesLeft(), robot.getYaw());
                break;

            case 22:
                straightaway(robot, robot.getYaw(), robot.getDistanceInchesLeft(), 24);
                break;
            case 23: //PID reset for forwards roll five (to A9)
                startingDistance = robot.getDistanceInchesRight();
                startingYaw = 0;
                PIDBounce.reset();
                outerRadius = 44; //calculated, needs testing
                outerArcLength = (Math.PI * outerRadius) / 2;
                PIDBounce.disableContinuousInput();
                autonomousStep += 1;
                break;

            case 24: //PID reset for bounce to A9
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / (180 * Math.PI));
                currentDistance = robot.getDistanceInchesRight();
                correction = PIDBounce.calculate(outerRadius * yawDifference + (robot.getDistanceInchesRight() - startingDistance));
                robot.setMotorPowerPercentage(defaultSpeed * (1 + correction), defaultSpeed * (1 - correction));
                if (currentDistance - startingDistance > outerArcLength) {
                    currentYaw = robot.getYaw();
                    autonomousStep += 1;
                }
                break;

            case 25: //PID reset for forwards roll five (to A9)
                startingDistance = robot.getDistanceInchesLeft();
                startingYaw = -90;
                PIDBounce.reset();
                PIDBounce.enableContinuousInput(-180, 180);
                autonomousStep += 1;
                break;
            case 26: //forwards roll five (to A9)
                correction = PIDBounce.calculate(robot.getYaw() - startingYaw);
                robot.setMotorPowerPercentage(straightSpeedScaling*defaultSpeed * (1 + correction), straightSpeedScaling*defaultSpeed * (1 - correction));
                currentDistance = robot.getDistanceInchesLeft();
                if (currentDistance - startingDistance > 97) { //6 ft > needs testing
                    autonomousStep += 1;
                }
                break;

            case 27:
                straightReset(-90, robot.getDistanceInchesLeft(), robot.getYaw());
                break;

            case 28: //backwards straight
                correction = PIDBounce.calculate(robot.getYaw() - startingYaw);
                robot.setMotorPowerPercentage(-straightSpeedScaling*defaultSpeed * (1 - correction), -straightSpeedScaling*defaultSpeed * (1 + correction));
                currentDistance = robot.getDistanceInchesLeft();
                if (currentDistance - startingDistance < -12) { //6 ft
                    autonomousStep += 1;
                }
                break;

            case 29: //PID reset for arc to finish
                startingDistance = robot.getDistanceInchesLeft();
                startingYaw = -90;
                PIDBounce.reset();
                outerRadius =  44; //calculated, needs testing
                outerArcLength = (Math.PI * outerRadius)/2;
                PIDBounce.disableContinuousInput();
                autonomousStep += 1;
                break;

            case 30: //final arc to finish zone
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / (180 * Math.PI));
                currentDistance = robot.getDistanceInchesLeft();
                correction = PIDBounce.calculate(outerRadius * yawDifference - (robot.getDistanceInchesLeft() - startingDistance));
                robot.setMotorPowerPercentage(-outerPowerScaling*defaultSpeed * (1 - correction), -innerPowerScaling*defaultSpeed * (1 + correction));
                if (currentDistance - startingDistance < -outerArcLength) {
                    currentYaw = robot.getYaw();
                    autonomousStep += 1;
                }
                break;
            case 31:
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
