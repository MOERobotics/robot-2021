package frc.robot.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.GenericRobot;

public class AutoNavSlalom extends GenericAutonomous {

    //change speed depending on robot!! (CaMOElot = .4, TestBot = .3)
    double defaultSpeed = 0.25;

    static double startingYaw = 0.0;
    static double startingDistance = 0.0;
    double correction;
    static double currentYaw = 0;
    double circumference;
    double yawDifference;
    long startingTime;
    double circumferenceQuarter;
    double localStartDistance; //how far overshot on loop thirds
    double smartSpeedCoeff;
    double deltaDistance;
    double goalYaw;

    double outerRadius = 48; //explicitly for S-turns
    double outerArcLength = 69.7; //explicitly for S-turns

    double semiCircleOuterRadius = 47; //turning radius + wheelbase (28")

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

        double currentDistance = 0;
        double yawError;
        switch (autonomousStep) {
            case -1: //resets navx, encoders, PID and waits (first arc reset (1/2))

                PIDSteering.reset();
                PIDSteering.disableContinuousInput();
                robot.resetAttitude();
                robot.resetEncoders();

                currentYaw = 0;
                if (System.currentTimeMillis() >= startingTime + 100) {
                    autonomousStep += 1;
                }
                break;

            case 0: //first (LEFT) arc (1/2) to D4
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                currentDistance = robot.getDistanceInchesRight();
                correction = PIDSteering.calculate(-(currentDistance - startingDistance) + outerRadius * yawDifference);
                robot.setMotorPowerPercentage((defaultSpeed * .75) * (1 + correction), (defaultSpeed * 1.5) * (1 - correction));
                if (currentDistance - startingDistance > outerArcLength) {
                    autonomousStep += 1;
                }
                break;

            case 1: //first arc reset (2/2)
                PIDSteering.reset();
                PIDSteering.disableContinuousInput();
                startingDistance = robot.getDistanceInchesLeft();
                startingYaw = robot.getYaw();
                autonomousStep += 1;
                break;

            case 2: //first (RIGHT) arc (2/2) to D4
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                currentDistance = robot.getDistanceInchesLeft();
                correction = PIDSteering.calculate((currentDistance - startingDistance) + outerRadius * yawDifference);
                robot.setMotorPowerPercentage((defaultSpeed * 1.5) * (1 + correction), (defaultSpeed * .75) * (1 - correction));
                if (currentDistance - startingDistance > outerArcLength) {
                    autonomousStep += 1;
                }
                break;

            case 3: //first straightaway reset
                PIDSteering.reset();
                PIDSteering.enableContinuousInput(-180, 180);

                currentYaw = 0;
                startingDistance = robot.getDistanceInchesLeft(); //set starting distance prior to circumference path
                startingYaw = robot.getYaw();

                circumference = (2 * Math.PI * semiCircleOuterRadius); //1/2 of a loop

                autonomousStep += 1;
                break;

            case 4: //first straightaway (120in)
                correction = PIDSteering.calculate(robot.getYaw() - currentYaw);
                robot.setMotorPowerPercentage(defaultSpeed * (1 + correction), defaultSpeed * (1 - correction));

                currentDistance = robot.getDistanceInchesLeft();

                if (currentDistance - startingDistance > 120) {
                    autonomousStep += 1;
                }
                break;

            case 5: //reset for second (right) arc (1/2)
                PIDSteering.reset();
                PIDSteering.disableContinuousInput();
                startingDistance = robot.getDistanceInchesLeft();
                startingYaw = robot.getYaw();
                autonomousStep += 1;
                break;

            case 6: //second (right) arc (1/2)
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                currentDistance = robot.getDistanceInchesLeft();
                correction = PIDSteering.calculate((currentDistance - startingDistance) + outerRadius * yawDifference);
                robot.setMotorPowerPercentage((defaultSpeed * 1.5) * (1 + correction), (defaultSpeed * .75) * (1 - correction));
                if (currentDistance - startingDistance > outerArcLength) {
                    autonomousStep += 1;
                }
                break;

            case 7: //reset for second (left) arc (2/2)
                PIDSteering.reset();
                PIDSteering.disableContinuousInput();
                startingDistance = robot.getDistanceInchesLeft();
                startingYaw = robot.getYaw();
                autonomousStep += 1;
                break;

            case 8: //second (left) arc (2/2)
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                currentDistance = robot.getDistanceInchesRight();
                correction = PIDSteering.calculate((currentDistance - startingDistance) + outerRadius * yawDifference);
                robot.setMotorPowerPercentage((defaultSpeed * .75) * (1 + correction), (defaultSpeed * 1.5) * (1 - correction));
                if (currentDistance - startingDistance > outerArcLength) {
                    autonomousStep += 1;
                }
                break;

            case 9: //reset for semicircle (1/2)
                PIDSteering.reset();
                PIDSteering.disableContinuousInput();

                startingDistance = robot.getDistanceInchesRight(); //set starting distance prior to circumference path
                startingYaw = robot.getYaw();

                circumference = (2 * Math.PI * semiCircleOuterRadius); //calculate circumference 2pir (inner or outer radius)

                autonomousStep += 1;
                break;

            case 10: //semicircle (1/2) (left arc) **CAN BE DONE IN ONE MANEUVER??**
                circumferenceQuarter = circumference / 4;
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                currentDistance = robot.getDistanceInchesRight();
                correction = PIDSteering.calculate(semiCircleOuterRadius * yawDifference - (currentDistance - startingDistance));
                robot.setMotorPowerPercentage((defaultSpeed * .75) * (1 + correction), (defaultSpeed * 1.5) * (1 - correction));
                if (currentDistance - startingDistance > circumferenceQuarter) {
                    autonomousStep += 1;
                }
                break;

            case 11: //reset for semicircle (2/2)
                localStartDistance = robot.getDistanceInchesRight();
                startingYaw = robot.getYaw();
                autonomousStep += 1;
                break;

            case 12: //semicircle (2/2) (left arc)
                circumferenceQuarter = circumference / 4;
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                currentDistance = robot.getDistanceInchesRight();
                correction = PIDSteering.calculate(semiCircleOuterRadius * yawDifference - (currentDistance - localStartDistance));
                robot.setMotorPowerPercentage((defaultSpeed * .75) * (1 + correction), (defaultSpeed * 1.5) * (1 - correction));

                if (currentDistance - localStartDistance > circumferenceQuarter) {
                    autonomousStep += 1;
                }
                break;

            //---------------- semicircle complete...end of first half ----------------

            case 13: //reset for left arc
                PIDSteering.reset();
                PIDSteering.disableContinuousInput();
                startingDistance = robot.getDistanceInchesRight();
                startingYaw = robot.getYaw();
                autonomousStep += 1;
                break;

            case 14: //left arc
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                currentDistance = robot.getDistanceInchesRight();
                correction = PIDSteering.calculate((currentDistance - startingDistance) + outerRadius * yawDifference);
                robot.setMotorPowerPercentage((defaultSpeed * .75) * (1 + correction), (defaultSpeed * 1.5) * (1 - correction));
                if (currentDistance - startingDistance > outerArcLength) {
                    autonomousStep += 1;
                }
                break;

            case 15: //reset for right arc
                PIDSteering.reset();
                PIDSteering.disableContinuousInput();
                startingDistance = robot.getDistanceInchesLeft();
                startingYaw = robot.getYaw();
                autonomousStep += 1;
                break;

            case 16: //right arc
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                currentDistance = robot.getDistanceInchesLeft();
                correction = PIDSteering.calculate((currentDistance - startingDistance) + outerRadius * yawDifference);
                robot.setMotorPowerPercentage((defaultSpeed * 1.5) * (1 + correction), (defaultSpeed * .75) * (1 - correction));
                if (currentDistance - startingDistance > outerArcLength) {
                    autonomousStep += 1;
                }
                break;

            case 17: //stop (for testing purposes)
                robot.driveForward(0);
                break;


        }
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

