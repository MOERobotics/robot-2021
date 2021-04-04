package frc.robot.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.GenericRobot;

public class AutoNavBarrel extends GenericAutonomous {

    //change speed depending on robot!! (CaMOElot = .4, TestBot = .3)
    double currentDistance = 0;
    double defaultSpeed = .25;
    static double startingYaw = 0.0;
    double startingDistance = 0.0;
    double correction;
    static double currentYaw = 0;
    double circumference;
    double yawDifference;
    long startingTime;
    double circumferenceThird;
    double localStartDistance; //how far overshot on loop thirds
    double smartSpeedCoeff;
    double deltaDistance;
    double leftPower;
    double rightPower;
    double goalYaw;

    double outerRadius = 40; //turning radius + wheelbase (28")

    PIDController PIDSteering;

    @Override
    public void autonomousInit(GenericRobot robot) {
        startingTime = System.currentTimeMillis();
        PIDSteering = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
        startingDistance = 0;
        autonomousStep = -1;
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        SmartDashboard.putNumber("Autostep", autonomousStep); //7.5' to second loop

        double yawError;
        switch (autonomousStep) {
            case -1: //resets navx, encoders, PID and waits
                autoInitiate(robot);
                break;

            case 0: //straight to first loop (94in)
                straightaway(robot, 104, defaultSpeed);
                break;

            case 1: //first loop reset (1/3)
                //PIDSteering.reset();
                //PIDSteering.disableContinuousInput();

                //startingDistance = robot.getDistanceInchesLeft(); //set starting distance prior to circumference path
                //startingYaw = robot.getYaw();

                circumference = 2 * Math.PI * outerRadius; //calculate circumference 2pir (inner or outer radius)

                //autonomousStep += 1;
                arcReset(robot, "right");
                break;

            case 2: //first loop (1/3)
                circumferenceThird = circumference / 3; //first third
                //yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                //currentDistance = robot.getDistanceInchesLeft();
                //correction = PIDSteering.calculate(firstLoopOuterRadius * yawDifference - (robot.getDistanceInchesLeft() - startingDistance));
                //leftPower = (defaultSpeed*1.5) * (1 + correction);
                //rightPower = (defaultSpeed*.75) * (1 - correction);

                /*
                if (currentDistance - startingDistance > circumferenceThird) { autonomousStep += 1;} //loop complete
                 */


                arc(robot, "right", circumferenceThird, defaultSpeed);
                break;

            case 3: //first loop reset (2/3)
                localStartDistance = robot.getDistanceInchesLeft();
                startingYaw = robot.getYaw();
                autonomousStep += 1;
                break;

            case 4: //first loop (2/3)
                circumferenceThird = 2 * circumference / 3; //second third

                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                currentDistance = robot.getDistanceInchesLeft();

                correction = PIDSteering.calculate(outerRadius * yawDifference - (robot.getDistanceInchesLeft() - localStartDistance));
                leftPower = (defaultSpeed*1.5) * (1 + correction);
                rightPower = (defaultSpeed*.75) * (1 - correction);

                if (currentDistance - startingDistance > circumferenceThird) { //loop complete
                    autonomousStep += 1;
                }
                break;

            case 5: //first loop reset (3/3)
                localStartDistance = robot.getDistanceInchesLeft();
                startingYaw = robot.getYaw();
                autonomousStep += 1;
                break;

            case 6: //first loop (3/3)
                circumferenceThird = circumference; //final third

                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                currentDistance = robot.getDistanceInchesLeft();

                correction = PIDSteering.calculate(outerRadius * yawDifference - (robot.getDistanceInchesLeft() - localStartDistance));
                leftPower = (defaultSpeed*1.5) * (1 + correction);
                rightPower = (defaultSpeed*.75) * (1 - correction);

                if (currentDistance - startingDistance > circumference) { //loop complete
                    autonomousStep += 1;
                }
                break;

            case 7: //post first loop PID reset
                //leftPower = 0;
                //rightPower = 0;
                //PIDSteering.reset();
                //PIDSteering.enableContinuousInput(-180, 180);
                //startingDistance = robot.getDistanceInchesRight();
                //currentDistance = startingDistance;
                //currentYaw = 0;
                //autonomousStep += 1;
                straightReset(robot, 0);
                break;

            case 8: //straightaway from first loop to second loop (80 inches)
                //correction = PIDSteering.calculate(robot.getYaw() - currentYaw);
                //leftPower = defaultSpeed *(1+ correction);
                //rightPower = defaultSpeed *(1- correction);

                //currentDistance = robot.getDistanceInchesLeft();
                //currentDistance = robot.getDistanceInchesRight();

                //double a = currentDistance - startingDistance;
                //if (a > 120) autonomousStep += 1;
                straightaway(robot, 105, defaultSpeed);
                break;
            case 9: //second loop reset (1/?) (7/8th of a loop)
                //PIDSteering.reset();
                //PIDSteering.disableContinuousInput();

                //startingDistance = robot.getDistanceInchesRight(); //set starting distance prior to circumference path
                //startingYaw = robot.getYaw();

                circumference = 2 * Math.PI * outerRadius * 0.875; //(7/8 of a loop)

                //autonomousStep += 1;
                arcReset(robot, "left");
                break;
            case 10: //second loop (1/3)
                circumferenceThird = circumference / 3; //first third
                //yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                //currentDistance = robot.getDistanceInchesRight();
                //correction = PIDSteering.calculate(outerRadius * yawDifference + (robot.getDistanceInchesRight() - startingDistance));
                //leftPower = (defaultSpeed*.75) * (1 + correction);
                //rightPower = defaultSpeed*1.5 * (1 - correction);

                //if (currentDistance - startingDistance > circumferenceThird) autonomousStep += 1; //loop complete

                arc(robot, "left", circumferenceThird, defaultSpeed);
                break;

            case 11: //second loop reset (2/3)
                localStartDistance = robot.getDistanceInchesRight();
                startingYaw = robot.getYaw();
                autonomousStep += 1;
                break;

            case 12: //second loop (2/3)
                circumferenceThird = 2 * circumference / 3; //second third

                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                currentDistance = robot.getDistanceInchesRight();

                correction = PIDSteering.calculate(outerRadius * yawDifference + (robot.getDistanceInchesRight() - localStartDistance));
                leftPower = defaultSpeed*.75 * (1 + correction);
                rightPower = defaultSpeed*1.5 * (1 - correction);

                if (currentDistance - startingDistance > circumferenceThird) autonomousStep += 1; //loop complete
                break;

            case 13: //second loop reset (3/3)
                localStartDistance = robot.getDistanceInchesRight();
                startingYaw = robot.getYaw();
                autonomousStep += 1;
                break;

            case 14: //second loop (3/3)
                circumferenceThird = 2 * circumference / 3; //final third

                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                currentDistance = robot.getDistanceInchesRight();

                correction = PIDSteering.calculate(outerRadius * yawDifference + (robot.getDistanceInchesRight() - localStartDistance));
                leftPower = defaultSpeed*.75 * (1 + correction);
                rightPower = defaultSpeed*.5 * (1 - correction);

                if (currentDistance - startingDistance > circumference) { //loop complete
                    autonomousStep += 1;
                }
                break;

            case 15: //post second loop PID reset
                //PIDSteering.reset();
                //PIDSteering.enableContinuousInput(-180, 180);
                //startingDistance = robot.getDistanceInchesRight();
                //currentYaw = 45;
                //autonomousStep += 1;
                straightReset(robot, 45);
                break;

            case 16: //straight to third loop (82in)
                //correction = PIDSteering.calculate(robot.getYaw() - currentYaw);

                //leftPower = defaultSpeed * (1 + correction);
                //rightPower = defaultSpeed * (1 - correction);
                //currentDistance = robot.getDistanceInchesRight();

                //if (currentDistance - startingDistance > 82) autonomousStep += 1;
                straightaway(robot, 82, defaultSpeed);
                break;
            case 17: //third loop reset (1/?) (5/8th of a loop)
                // THIRD LOOP UNTESTED AND UNREVIEWED

                PIDSteering.reset();
                PIDSteering.disableContinuousInput();

                startingDistance = robot.getDistanceInchesRight(); //set starting distance prior to circumference path
                startingYaw = robot.getYaw();

                circumference = 2 * Math.PI * outerRadius * 0.625; //(5/8 of a loop)

                autonomousStep += 1;
                arcReset(robot, "left");
                break;
            case 18: //third loop (1/3)
                circumferenceThird = circumference / 3; //first third
                //yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                //currentDistance = robot.getDistanceInchesRight();
                //correction = PIDSteering.calculate(outerRadius * yawDifference + (robot.getDistanceInchesRight() - startingDistance));
                //leftPower = defaultSpeed*.75 * (1 + correction);
                //rightPower = defaultSpeed*1.5 * (1 - correction);

                //if (currentDistance - startingDistance > circumferenceThird) autonomousStep += 1;
                arc(robot, "left", circumferenceThird, defaultSpeed);
                break;

            case 19: //third loop reset (2/3)
                localStartDistance = robot.getDistanceInchesRight();
                startingYaw = robot.getYaw();
                autonomousStep += 1;
                break;

            case 20: //third loop (2/3)
                circumferenceThird = 2 * circumference / 3; //second third

                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                currentDistance = robot.getDistanceInchesRight();

                correction = PIDSteering.calculate(outerRadius * yawDifference + (robot.getDistanceInchesRight() - localStartDistance));
                leftPower = defaultSpeed*.75 * (1 + correction);
                rightPower = defaultSpeed*1.5 * (1 - correction);

                if (currentDistance - startingDistance > circumferenceThird) { //loop complete
                    autonomousStep += 1; //NOTE: SET TO STOP
                }
                break;

            case 21: //third loop reset (3/3)
                localStartDistance = robot.getDistanceInchesRight();
                startingYaw = robot.getYaw();
                autonomousStep += 1;
                break;

            case 22: //third loop (3/3)
                circumferenceThird = 2 * circumference / 3; //final third

                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                currentDistance = robot.getDistanceInchesRight();

                correction = PIDSteering.calculate(outerRadius * yawDifference + (robot.getDistanceInchesRight() - localStartDistance));
                leftPower = defaultSpeed*.75 * (1 + correction);
                rightPower = defaultSpeed*1.5 * (1 - correction);

                if (currentDistance - startingDistance > circumference) { //loop complete
                    autonomousStep += 1;
                }
                break;

            case 23: //post third loop PID reset
                //PIDSteering.reset();
                //PIDSteering.enableContinuousInput(-180, 180);
                //startingDistance = robot.getDistanceInchesRight();
                //currentYaw = 180;
                //autonomousStep += 1;
                straightReset(robot, 180);
                break;

            case 24: //straight to end (272in)
                //correction = PIDSteering.calculate(robot.getYaw() - currentYaw);

                //leftPower = defaultSpeed * (1 + correction);
                //rightPower = defaultSpeed * (1 - correction);
                //currentDistance = robot.getDistanceInchesRight();

                //if (currentDistance - startingDistance > 272) autonomousStep += 1;
                straightaway(robot, 272, defaultSpeed);
                break;
            case 25: //temporary stop (for testing purposes only)
                robot.driveForward(0);
                break;


        }

        robot.setMotorPowerPercentage(leftPower, rightPower);


    }
    //Methods for common autonomous routine maneuvers
    public void autoInitiate(GenericRobot robot) {
        PIDSteering.reset();
        PIDSteering.disableContinuousInput();
        robot.resetAttitude();
        robot.resetEncoders();

        currentYaw = 0;
        if (System.currentTimeMillis() >= startingTime + 100) autonomousStep += 1;
    }

    public void arcReset(GenericRobot robot, String direction) {
        PIDSteering.reset();
        PIDSteering.disableContinuousInput();

        if (direction.equalsIgnoreCase("left")) {
            startingDistance = robot.getDistanceInchesRight();
        } else if (direction.equalsIgnoreCase("right")) {
            startingDistance = robot.getDistanceInchesLeft();
        } else { //we got problems
            System.out.println("ERROR: Define left or right arcReset");
        }
        startingYaw = robot.getYaw();

        autonomousStep += 1;
    }

    public void arc(GenericRobot robot, String direction, double outArcLength, double desiredSpeed) {
        yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);

        if (direction.equalsIgnoreCase("left")) {
            currentDistance = robot.getDistanceInchesRight();
            correction = PIDSteering.calculate((currentDistance - startingDistance) + outerRadius * yawDifference);
            robot.setMotorPowerPercentage((desiredSpeed * .75) + correction, (desiredSpeed * 1.5) - correction);
        } else if (direction.equalsIgnoreCase("right")) {
            currentDistance = robot.getDistanceInchesLeft();
            correction = PIDSteering.calculate(-(currentDistance - startingDistance) + outerRadius * yawDifference);
            robot.setMotorPowerPercentage((desiredSpeed * 1.5) + correction, (desiredSpeed * .75) - correction);
        } else { //we got problems
            System.out.println("ERROR: Define left or right arc");
        }

        if (currentDistance - startingDistance > outArcLength) autonomousStep += 1;
    }

    public void straightReset(GenericRobot robot, double optimalYaw) {
        PIDSteering.reset();
        PIDSteering.enableContinuousInput(-180, 180);

        currentYaw = optimalYaw; //0 if facing forward, 180 if facing backward (relative to starting position)
        startingDistance = robot.getDistanceInchesLeft();

        autonomousStep += 1;
    }


    public void straightaway(GenericRobot robot, double finalDistance, double desiredSpeed) { //desiredSpeed can be set to defaultSpeed
        correction = PIDSteering.calculate(robot.getYaw() - currentYaw);
        robot.setMotorPowerPercentage((desiredSpeed + correction), (desiredSpeed - correction));

        currentDistance = robot.getDistanceInchesLeft();

        if (currentDistance - startingDistance > finalDistance) autonomousStep += 1;
    }

    //--------old methods--------
    /*
     public void leftArcReset(GenericRobot robot) {
        PIDSteering.reset();
        PIDSteering.disableContinuousInput();
        startingDistance = robot.getDistanceInchesRight();
        startingYaw = robot.getYaw();

        autonomousStep += 1;
    }

    public void rightArcReset(GenericRobot robot) {
        PIDSteering.reset();
        PIDSteering.disableContinuousInput();
        startingDistance = robot.getDistanceInchesLeft();
        startingYaw = robot.getYaw();

        autonomousStep += 1;
    }

    public void arcLeft(GenericRobot robot, double outArcLength) {
        yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
        currentDistance = robot.getDistanceInchesRight();
        correction = PIDSteering.calculate((currentDistance - startingDistance) + outerRadius * yawDifference);
        robot.setMotorPowerPercentage((defaultSpeed * .75) + correction, (defaultSpeed * 1.5) - correction);

        if (currentDistance - startingDistance > outArcLength)
            autonomousStep += 1;
    }

    public void arcRight(GenericRobot robot, double outArcLength) {
        yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
        currentDistance = robot.getDistanceInchesLeft();
        correction = PIDSteering.calculate(-(currentDistance - startingDistance) + outerRadius * yawDifference);
        robot.setMotorPowerPercentage((defaultSpeed * 1.5) + correction, (defaultSpeed * .75) - correction);

        if (currentDistance - startingDistance > outArcLength)
            autonomousStep += 1;
    }
     */


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


