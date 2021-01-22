package frc.robot.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.commands.GenericCommand;
import frc.robot.commands.LimelightAlign;
import frc.robot.genericrobot.GenericRobot;

import static frc.robot.Util.speedScale;

public class AutonavLoop extends GenericAutonomous {

    //change speed depending on robot!! (CaMOElot = .4, TestBot = .3)
    double defaultSpeed = 0.25;

    static double startingYaw = 0.0;
    static double startingDistance = 0.0;
    double correction;
    static double currentYaw = 0;
    double innerRadius = 0;
    double circumference;
    double yawDifference = 0;
    long startingTime;
    double powerDecrement;
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

            case 1: //PID reset for loop
                PIDSteering.reset();
                PIDSteering.disableContinuousInput();

                startingDistance = robot.getDistanceInchesLeft(); //set starting distance prior to circumference path
                startingYaw = robot.getYaw();

                circumference = 2 * Math.PI * innerRadius; //calculate circumference 2pir (inner or outer radius)
                autonomousStep += 1;
                break;

            case 2: //loop

                yawDifference = continuousAngleDiff((robot.getYaw() - startingYaw) / 180 * Math.PI);
                currentDistance = robot.getDistanceInchesLeft();

                correction = PIDSteering.calculate(innerRadius * yawDifference - (robot.getDistanceInchesLeft() - startingDistance));
                robot.setMotorPowerPercentage((defaultSpeed * 1.5) * (1 + correction), (defaultSpeed * .75) * (1 - correction));

                if (currentDistance - startingDistance > circumference) { //loop complete
                    autonomousStep += 1;
                }
                break;

            case 3: //after loop, PID reset for final 5ft
                PIDSteering.reset();
                PIDSteering.enableContinuousInput(-180, 180);
                startingDistance = robot.getDistanceInchesLeft();
                currentYaw = 0;
                autonomousStep += 1;
                break;

            case 4: //final 5ft straightaway
                correction = PIDSteering.calculate(robot.getYaw() - currentYaw);

                //accelerate?

                robot.setMotorPowerPercentage(defaultSpeed * (1.0 + correction), defaultSpeed * (1 - correction));
                currentDistance = robot.getDistanceInchesLeft();

                //decelerate as approach 5ft?


                if (currentDistance - startingDistance > 60) {
                    autonomousStep += 1;
                }
                break;

            case 5: //cease thy autonomous
                robot.setMotorPowerPercentage(0, 0);
                break;

        }


    }
}


