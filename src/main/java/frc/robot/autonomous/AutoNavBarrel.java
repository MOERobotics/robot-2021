package frc.robot.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.GenericRobot;
import java.io.*;

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

    double firstLoopOuterRadius = 44; //turning radius + wheelbase (28")
    double secondLoopOuterRadius = 44;

    double innerPower;
    double outerPower;

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

        outerPower = Math.sqrt(firstLoopOuterRadius/(firstLoopOuterRadius-28));
        innerPower = 1/outerPower;

        double yawError;
        switch (autonomousStep) {
            case -1: //resets navx, encoders, PID and waits

                PIDSteering.reset();
                PIDSteering.enableContinuousInput(-180, 180);
                robot.resetAttitude();
                robot.resetEncoders();

                currentYaw = 0;
                if (System.currentTimeMillis() >= startingTime + 100) {
                    autonomousStep += 1;
                }
                break;

            case 0: //straight to first loop (94in)
                correction = PIDSteering.calculate(robot.getYaw() - currentYaw);

                leftPower = defaultSpeed * (1 + correction);
                rightPower = defaultSpeed * (1 - correction);
                currentDistance = robot.getDistanceInchesLeft();

                if (currentDistance - startingDistance > 100) {
                    autonomousStep += 1;
                }
                break;

            case 1: //first loop reset (1/3)
                PIDSteering.reset();
                PIDSteering.disableContinuousInput();

                startingDistance = robot.getDistanceInchesLeft(); //set starting distance prior to circumference path
                startingYaw = robot.getYaw();

                circumference = 2 * Math.PI * firstLoopOuterRadius; //calculate circumference 2pir (inner or outer radius)

                autonomousStep += 1;

                break;

            case 2: //first loop (1/3)
                circumferenceThird = circumference / 3; //first third
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                currentDistance = robot.getDistanceInchesLeft();
                correction = PIDSteering.calculate(firstLoopOuterRadius * yawDifference - (robot.getDistanceInchesLeft() - startingDistance));
                leftPower = (defaultSpeed*outerPower) * (1 + correction);
                rightPower = (defaultSpeed*innerPower) * (1 - correction);

                if (currentDistance - startingDistance > circumferenceThird) { //loop complete
                    autonomousStep += 1;
                }
                break;

            case 3: //first loop reset (2/3)
                localStartDistance = robot.getDistanceInchesLeft();
                startingYaw = robot.getYaw();
                autonomousStep += 1;
                break;

            case 4: //first loop (2/3)
                circumferenceThird = 2*circumference / 3; //second third

                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                currentDistance = robot.getDistanceInchesLeft();

                correction = PIDSteering.calculate(firstLoopOuterRadius * yawDifference - (robot.getDistanceInchesLeft() - localStartDistance));
                leftPower = (defaultSpeed*outerPower) * (1 + correction);
                rightPower = (defaultSpeed*innerPower) * (1 - correction);

                if (currentDistance - startingDistance > circumferenceThird) { //loop complete
                    autonomousStep += 1; //NOTE: SET TO STOP
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

                correction = PIDSteering.calculate(firstLoopOuterRadius * yawDifference - (robot.getDistanceInchesLeft() - localStartDistance));
                leftPower = (defaultSpeed*outerPower) * (1 + correction);
                rightPower = (defaultSpeed*innerPower) * (1 - correction);

                if (currentDistance - startingDistance > circumference) { //loop complete
                    autonomousStep += 1;
                }
                break;

            case 7: //post first loop PID reset
                //leftPower = 0;
                //rightPower = 0;
                PIDSteering.reset();
                PIDSteering.enableContinuousInput(-180, 180);
                startingDistance = robot.getDistanceInchesRight();
                currentDistance = startingDistance;
                currentYaw = 0;
                autonomousStep += 1;
                break;

            case 8: //straightaway from first loop to second loop (80 inches)
                correction = PIDSteering.calculate(robot.getYaw() - currentYaw);
                leftPower = defaultSpeed *(1+ correction);
                rightPower = defaultSpeed *(1- correction);

                //currentDistance = robot.getDistanceInchesLeft();
                currentDistance = robot.getDistanceInchesRight();

                double a = currentDistance - startingDistance;
                if (a > 100) {
                    autonomousStep += 1;
                }
                break;
            case 9: //second loop reset (1/?) (7/8th of a loop)

                PIDSteering.reset();
                PIDSteering.disableContinuousInput();

                startingDistance = robot.getDistanceInchesRight(); //set starting distance prior to circumference path
                startingYaw = robot.getYaw();

                circumference = 2 * Math.PI * secondLoopOuterRadius * 0.875; //(7/8 of a loop)

                autonomousStep += 1;
                break;
            case 10: //second loop (1/3)
                circumferenceThird = circumference / 3; //first third
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                currentDistance = robot.getDistanceInchesRight();
                correction = PIDSteering.calculate(firstLoopOuterRadius * yawDifference + (robot.getDistanceInchesRight() - startingDistance));
                leftPower = (defaultSpeed*innerPower) * (1 + correction);
                rightPower = defaultSpeed*outerPower * (1 - correction);

                if (currentDistance - startingDistance > circumferenceThird) { //loop complete
                    autonomousStep += 1;
                }
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

                correction = PIDSteering.calculate(firstLoopOuterRadius * yawDifference + (robot.getDistanceInchesRight() - localStartDistance));
                leftPower = defaultSpeed*innerPower * (1 + correction);
                rightPower = defaultSpeed*outerPower * (1 - correction);

                if (currentDistance - startingDistance > circumferenceThird) { //loop complete
                    autonomousStep += 1; //NOTE: SET TO STOP
                }
                break;

            case 13: //second loop reset (3/3)
                localStartDistance = robot.getDistanceInchesRight();
                startingYaw = robot.getYaw();
                autonomousStep += 1;
                break;

            case 14: //second loop (3/3)
                circumferenceThird = circumference; //final third

                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                currentDistance = robot.getDistanceInchesRight();

                correction = PIDSteering.calculate(firstLoopOuterRadius * yawDifference + (robot.getDistanceInchesRight() - localStartDistance));
                leftPower = defaultSpeed*innerPower * (1 + correction);
                rightPower = defaultSpeed*outerPower * (1 - correction);

                if (currentDistance - startingDistance > circumference) { //loop complete
                    autonomousStep += 1;
                }
                break;

            case 15: //post second loop PID reset
                PIDSteering.reset();
                PIDSteering.enableContinuousInput(-180, 180);
                startingDistance = robot.getDistanceInchesRight();
                currentYaw = 45;
                autonomousStep += 1;
                break;

            case 16: //straight to third loop (82in)
                correction = PIDSteering.calculate(robot.getYaw() - currentYaw);

                leftPower = defaultSpeed * (1 + correction);
                rightPower = defaultSpeed * (1 - correction);
                currentDistance = robot.getDistanceInchesRight();

                if (currentDistance - startingDistance > 104) {
                    autonomousStep += 1;
                }
                break;
            case 17: //third loop reset (1/?) (5/8th of a loop)
                // THIRD LOOP UNTESTED AND UNREVIEWED

                PIDSteering.reset();
                PIDSteering.disableContinuousInput();

                startingDistance = robot.getDistanceInchesRight(); //set starting distance prior to circumference path
                startingYaw = robot.getYaw();

                circumference = 2 * Math.PI * secondLoopOuterRadius * 0.625; //(5/8 of a loop)

                autonomousStep += 1;
                break;
            case 18: //third loop (1/3)
                circumferenceThird = circumference / 3; //first third
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                currentDistance = robot.getDistanceInchesRight();
                correction = PIDSteering.calculate(firstLoopOuterRadius * yawDifference + (robot.getDistanceInchesRight() - startingDistance));
                leftPower = defaultSpeed*innerPower * (1 + correction);
                rightPower = defaultSpeed*outerPower * (1 - correction);

                if (currentDistance - startingDistance > circumferenceThird) { //loop complete
                    autonomousStep += 1;
                }
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

                correction = PIDSteering.calculate(firstLoopOuterRadius * yawDifference + (robot.getDistanceInchesRight() - localStartDistance));
                leftPower = defaultSpeed*innerPower * (1 + correction);
                rightPower = defaultSpeed*outerPower * (1 - correction);

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

                correction = PIDSteering.calculate(firstLoopOuterRadius * yawDifference + (robot.getDistanceInchesRight() - localStartDistance));
                leftPower = defaultSpeed*innerPower * (1 + correction);
                rightPower = defaultSpeed*outerPower * (1 - correction);

                if (currentDistance - startingDistance > circumference) { //loop complete
                    autonomousStep += 1;
                }
                break;

            case 23: //post third loop PID reset
                PIDSteering.reset();
                PIDSteering.enableContinuousInput(-180, 180);
                startingDistance = robot.getDistanceInchesRight();
                currentYaw = 180;
                autonomousStep += 1;
                break;

            case 24: //straight to end (272in)
                correction = PIDSteering.calculate(robot.getYaw() - currentYaw);

                leftPower = defaultSpeed * (1 + correction);
                rightPower = defaultSpeed * (1 - correction);
                currentDistance = robot.getDistanceInchesRight();

                if (currentDistance - startingDistance > 272) {
                    autonomousStep += 1;
                }
                break;
            case 25: //temporary stop (for testing purposes only)
                System.out.println(currentDistance);
                System.out.println(startingDistance);
                System.out.println(currentDistance-startingDistance);
                leftPower = 0;
                rightPower = 0;
                break;
        }

        robot.setMotorPowerPercentage(leftPower, rightPower);


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


