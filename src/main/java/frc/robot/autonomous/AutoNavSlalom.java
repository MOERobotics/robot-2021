package frc.robot.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.GenericRobot;

import java.io.IOException;
import java.util.Locale;

public class AutoNavSlalom extends GenericAutonomous {

    //change speed depending on robot!! (CaMOElot = .4, TestBot = .3)
    double defaultSpeed = 0.18;

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
    double currentDistance;

    double outerRadius = 48; //explicitly for S-turns
    double outerArcLength = 74; //69.7; //explicitly for S-turns

    double semiCircleOuterRadius = 44; //turning radius + wheelbase (28")

    PIDController PIDSteering;

    @Override
    public void autonomousInit(GenericRobot robot) {
        startingTime = System.currentTimeMillis();
        PIDSteering = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
        startingDistance = 0;
        autonomousStep = 0; //-1 for non-method, -30 for method
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        SmartDashboard.putNumber("Autostep", autonomousStep); //7.5' to second loop

        switch (autonomousStep) {

            case 0: //initiate
                autoInitiate(robot);
                break;

            case 1: //straightaway reset
                straightReset(robot, 0);
                break;

            case 2: //straightaway
                straightaway(robot, 16, defaultSpeed);
                break;
            case 3:
                //leftArcReset(robot);
                arcReset(robot, "Left");
                break;

            case 4: //s-turn 1/2
                //arcLeft(robot, outerArcLength);
                arc(robot, "left", outerArcLength, defaultSpeed);
                break;

            case 5: //straightaway reset
                straightReset(robot, -90);
                break;

            case 6: //straightaway
                straightaway(robot, 2, defaultSpeed);
                break;

            case 7: //s-turn reset
                //rightArcReset(robot);
                arcReset(robot, "right");
                break;

            case 8: //s-turn 2/2
                //arcRight(robot, outerArcLength);
                arc(robot, "right", outerArcLength, defaultSpeed);
                break;

            case 9: //straightaway reset
                straightReset(robot, 0);
                break;

            case 10: //straightaway
                straightaway(robot, 90, 0.5);
                break;

            case 11: //straightaway
                straightaway(robot, 118, 0.2);
                break;

            case 12: //s-turn pre-reset
                //rightArcReset(robot);
                arcReset(robot, "right");
                break;

            case 13: //s-turn 1/2
                //arcRight(robot, outerArcLength);
                arc(robot, "right", outerArcLength, defaultSpeed);
                break;

            case 14: //straightaway reset
                straightReset(robot, 90);
                break;

            case 15: //straightaway
                straightaway(robot, 2, defaultSpeed);
                break;

            case 16: //s-turn reset
                //leftArcReset(robot);
                arcReset(robot, "left");
                break;

            case 17: //s-turn 2/2
                //arcLeft(robot, outerArcLength);
                arc(robot, "left", outerArcLength, defaultSpeed);
                break;

            case 18: //semicircle pre-reset
                circumference = (2 * Math.PI * semiCircleOuterRadius); //calculate circumference 2pir (inner or outer radius)
                //leftArcReset(robot);
                arcReset(robot, "left");
                break;

            case 19: //semicircle 1/2
                circumferenceQuarter = circumference / 4;
                //arcLeft(robot, circumferenceQuarter);
                arc(robot, "left", circumferenceQuarter, defaultSpeed);
                break;

            case 20: //semicircle reset prep
                localStartDistance = robot.getDistanceInchesRight();
                startingYaw = robot.getYaw();
                autonomousStep += 1;
                break;

            case 21: //semicircle 2/2
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                currentDistance = robot.getDistanceInchesRight();
                correction = PIDSteering.calculate(semiCircleOuterRadius * yawDifference + (currentDistance - localStartDistance));
                robot.setMotorPowerPercentage((defaultSpeed * .75 + correction), (defaultSpeed * 1.5 - correction));

                if (currentDistance - localStartDistance > circumferenceQuarter) {
                    autonomousStep += 1;
                }
                break;

            case 22://reset
                straightReset(robot, 180);
                break;

            case 23: //straightaway
                straightaway(robot, 4, defaultSpeed);
                break;

            case 24: //s-turn pre-reset
                PIDSteering.reset();
                PIDSteering.disableContinuousInput();
                startingDistance = robot.getDistanceInchesRight();
                startingYaw = 180;

                autonomousStep += 1;
                break;

            case 25: //s-turn 1/2
                //arcLeft(robot, outerArcLength);
                arc(robot, "left", outerArcLength, defaultSpeed);
                break;

            case 26: //straightaway reset
                straightReset(robot, 90);
                break;

            case 27: //straightaway
                straightaway(robot, 2, defaultSpeed);
                break;

            case 28: //s-turn reset
                //rightArcReset(robot);
                arcReset(robot, "right");
                break;

            case 29: //s-turn 2/2
                //arcRight(robot, outerArcLength);
                arc(robot, "right", outerArcLength, defaultSpeed);
                break;

            case 30: //straightaway reset
                straightReset(robot, 180);
                break;

            case 31: //straightaway
                straightaway(robot, 100, 0.5);
                break;

            case 32: //straightaway
                straightaway(robot, 128, 0.2);
                break;

            case 33: //s-turn pre-reset
                //rightArcReset(robot);
                arcReset(robot, "right");
                break;

            case 34: //s-turn 1/2
                //arcRight(robot, outerArcLength);
                arc(robot, "right", outerArcLength, defaultSpeed);
                break;

            case 35: //s-turn reset
                //leftArcReset(robot);
                arcReset(robot, "left");
                break;

            case 36: //s-turn 1/2
                //arcLeft(robot, outerArcLength);
                arc(robot, "left", outerArcLength, defaultSpeed);
                break;

            case 37: //cease thine auto routine
                robot.driveForward(0);
                break;

            //Non-Methods moved below methods to prevent annoying scrolling
        }
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

    /* non-method version

    case -1: //resets navx, encoders, PID and waits (first arc reset (1/2))


                PIDSteering.reset();
                PIDSteering.disableContinuousInput();
                robot.resetAttitude();
                robot.resetEncoders();
                startingDistance = robot.getDistanceInchesRight();

                currentYaw = 0;
                if (System.currentTimeMillis() >= startingTime + 100) {
                    autonomousStep += 1;
                }
                break;

            case 0: //first (LEFT) arc (1/2) to D4
                yawDifference = continuousAngleDiff((robot.getYaw() - currentYaw) / 180 * Math.PI);
                currentDistance = robot.getDistanceInchesRight();
                correction = PIDSteering.calculate((currentDistance - startingDistance) + outerRadius * yawDifference);
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
                correction = PIDSteering.calculate(-(currentDistance - startingDistance) + outerRadius * yawDifference);
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
                correction = PIDSteering.calculate(-(currentDistance - startingDistance) + outerRadius * yawDifference);
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
                correction = PIDSteering.calculate(semiCircleOuterRadius * yawDifference + (currentDistance - startingDistance));
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
                correction = PIDSteering.calculate(semiCircleOuterRadius * yawDifference + (currentDistance - localStartDistance));
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
                correction = PIDSteering.calculate(-(currentDistance - startingDistance) + outerRadius * yawDifference);
                robot.setMotorPowerPercentage((defaultSpeed * 1.5) * (1 + correction), (defaultSpeed * .75) * (1 - correction));
                if (currentDistance - startingDistance > outerArcLength) {
                    autonomousStep += 1;
                }
                break;

            case 17: //reset for straightaway
                PIDSteering.reset();
                PIDSteering.enableContinuousInput(-180, 180);

                currentYaw = 180; //since straightaway is now facing 180deg
                startingDistance = robot.getDistanceInchesRight();
                startingYaw = robot.getYaw();

                autonomousStep += 1;
                break;

            case 18: //second straightaway
                correction = PIDSteering.calculate(robot.getYaw() - currentYaw);
                robot.setMotorPowerPercentage(defaultSpeed * (1 + correction), defaultSpeed * (1 - correction));

                currentDistance = robot.getDistanceInchesRight();

                if (currentDistance - startingDistance > 120) {
                    autonomousStep += 1;
                }
                break;

            case 19: //reset for right arc
                PIDSteering.reset();
                PIDSteering.disableContinuousInput();
                startingDistance = robot.getDistanceInchesLeft();
                startingYaw = robot.getYaw();
                autonomousStep += 1;
                break;

            case 20: //right arc
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                currentDistance = robot.getDistanceInchesLeft();
                correction = PIDSteering.calculate(-(currentDistance - startingDistance) + outerRadius * yawDifference);
                robot.setMotorPowerPercentage((defaultSpeed * 1.5) * (1 + correction), (defaultSpeed * .75) * (1 - correction));
                if (currentDistance - startingDistance > outerArcLength) {
                    autonomousStep += 1;

                }
                break;

            case 21: //reset for left arc
                PIDSteering.reset();
                PIDSteering.disableContinuousInput();
                startingDistance = robot.getDistanceInchesRight();
                startingYaw = robot.getYaw();
                autonomousStep += 1;
                break;

            case 22: //left arc
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                currentDistance = robot.getDistanceInchesRight();
                correction = PIDSteering.calculate((currentDistance - startingDistance) + outerRadius * yawDifference);
                robot.setMotorPowerPercentage((defaultSpeed * .75) * (1 + correction), (defaultSpeed * 1.5) * (1 - correction));
                if (currentDistance - startingDistance > outerArcLength) {
                    autonomousStep += 1;
                }
                break;

            case 23: //cease thine auto routine
                robot.driveForward(0);
                break;

*/
}


