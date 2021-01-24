package frc.robot.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.GenericCommand;
import frc.robot.commands.LimelightAlign;
import frc.robot.genericrobot.GenericRobot;

public class AutonavLoop extends GenericAutonomous {

    //change speed depending on robot!! (CaMOElot = .4, TestBot = .3)
    double defaultSpeed = 0.25;

    static double startingYaw = 0.0;
    static double startingDistance = 0.0;
    double correction;
    static double currentYaw = 0;
    double outerRadius = 64; //turning radius + wheelbase
    double circumference;
    double yawDifference;
    long startingTime;
    double powerDecrement;
    double circumferenceThird;
    double localStartDistance; //how far overshot on loop thirds

    PIDController PIDSteering;


    GenericCommand activeCommand = new LimelightAlign(-1.5, .8); //planA set setPoint to -2
    CollectPowerCells getCells = new CollectPowerCells();


    @Override
    public void autonomousInit(GenericRobot robot) {
        startingTime = System.currentTimeMillis();
        PIDSteering = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
        startingDistance = 0;
        autonomousStep = -1;
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        SmartDashboard.putNumber("Autostep", autonomousStep);

        double currentDistance = 0;
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

            case 0: //first forward 5ft
                correction = PIDSteering.calculate(robot.getYaw() - currentYaw);

                //accelerate?

                robot.setMotorPowerPercentage(defaultSpeed * (1 + correction), defaultSpeed * (1 - correction));
                currentDistance = robot.getDistanceInchesLeft();

                //decelerate as approach 5ft?

                if (currentDistance - startingDistance > 60) { //**NOTE: NEED TICKS TO INCH
                    autonomousStep += 1;
                }
                break;

            case 1: //PID reset for loop (1/3)
                PIDSteering.reset();
                PIDSteering.disableContinuousInput();

                startingDistance = robot.getDistanceInchesLeft(); //set starting distance prior to circumference path
                startingYaw = robot.getYaw();

                circumference = 2 * Math.PI * outerRadius; //calculate circumference 2pir (inner or outer radius)

                autonomousStep += 1; //**NOTE: INCREMENT BY 2 FOR TESTING BY THIRDS
                break;

            case 2: //loop 1/3
                circumferenceThird = circumference / 3; //first third
                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                currentDistance = robot.getDistanceInchesLeft();
                correction = PIDSteering.calculate(outerRadius * yawDifference - (robot.getDistanceInchesLeft() - startingDistance));
                robot.setMotorPowerPercentage((defaultSpeed * 1.5) * (1 + correction), (defaultSpeed * .75) * (1 - correction));

                if (currentDistance - startingDistance > circumferenceThird) { //loop complete
                    autonomousStep += 1;
                }
                break;

            case 3: //reset for loop 2/3
                localStartDistance = robot.getDistanceInchesLeft();
                startingYaw = robot.getYaw();
                autonomousStep += 1;
                break;


            case 4: //loop 2/3
                circumferenceThird = 2 * circumference / 3; //second third

                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                currentDistance = robot.getDistanceInchesLeft();

                correction = PIDSteering.calculate(outerRadius * yawDifference - (robot.getDistanceInchesLeft() - localStartDistance));
                robot.setMotorPowerPercentage((defaultSpeed * 1.5) * (1 + correction), (defaultSpeed * .75) * (1 - correction));

                if (currentDistance - startingDistance > circumferenceThird) { //loop complete
                    autonomousStep += 1; //NOTE: SET TO STOP
                }
                break;

            case 5: //reset for loop 3/3
                localStartDistance = robot.getDistanceInchesLeft();
                startingYaw = robot.getYaw();
                autonomousStep += 1;
                break;

            case 6: //loop 3/3
                circumferenceThird = 2 * circumference / 3; //final third

                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                currentDistance = robot.getDistanceInchesLeft();

                correction = PIDSteering.calculate(outerRadius * yawDifference - (robot.getDistanceInchesLeft() - localStartDistance));
                robot.setMotorPowerPercentage((defaultSpeed * 1.5) * (1 + correction), (defaultSpeed * .75) * (1 - correction));

                if (currentDistance - startingDistance > circumference) { //loop complete
                    autonomousStep += 1;
                }
                break;

            case 7: //after loop, PID reset for final 5ft
                PIDSteering.reset();
                PIDSteering.enableContinuousInput(-180, 180);
                startingDistance = robot.getDistanceInchesLeft();
                currentYaw = 0;
                autonomousStep += 1;
                break;

            case 8: //final 5ft straightaway
                correction = PIDSteering.calculate(robot.getYaw() - currentYaw);

                //accelerate?

                robot.setMotorPowerPercentage(defaultSpeed * (1.0 + correction), defaultSpeed * (1 - correction));
                currentDistance = robot.getDistanceInchesLeft();

                //decelerate as approach 5ft?


                if (currentDistance - startingDistance > 60) {
                    autonomousStep += 1;
                }
                break;

            case 9: //cease thy autonomous
                robot.setMotorPowerPercentage(0, 0);
                break;

        }


    }
    public double rightArcDiff(double deltaTheta){
        if(deltaTheta < 360){
            deltaTheta += 360;
        }
        deltaTheta = (deltaTheta * Math.PI) / 180;

        return deltaTheta;
    }
}


