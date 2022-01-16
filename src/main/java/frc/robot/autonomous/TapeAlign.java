package frc.robot.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.GenericRobot;

public class TapeAlign extends GenericAutonomous{

    double startAngle;

    double leftPower;
    double rightPower;
    double defaultPower = -.3;

    double startDistance;
    double differenceDistance;

    double sensorDist = 12.0;
    double theta = 0;

    double correction;
    double currentYaw;

    long startingTime = 0;

    public void autonomousInit(GenericRobot robot) {
        startingTime = System.currentTimeMillis();
        autonomousStep = -1;
    }

    public void autonomousPeriodic(GenericRobot robot){
        PIDController PIDSteering = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
        double yawError;
        SmartDashboard.putBoolean("MediumHigh", robot.getEscalatorSensorMediumHigh());
        SmartDashboard.putBoolean("Medium", robot.getEscalatorSensorMedium());
        SmartDashboard.putNumber("theta", theta);
        switch (autonomousStep) {
            case -1:
                robot.resetEncoders();
                robot.resetAttitude();

                if (System.currentTimeMillis() >= startingTime + 100) {
                    autonomousStep += 1;
                }
                break;
            case 0:
                startAngle = robot.getYaw();
                PIDSteering.reset();
                PIDSteering.enableContinuousInput(-180, 180);
                autonomousStep += 1;
                break;
            case 1:
                correction = PIDSteering.calculate(robot.getYaw() - startAngle);
                leftPower = defaultPower * (1 + correction);
                rightPower = defaultPower * (1 - correction);
                if (! robot.getEscalatorSensorMediumHigh()) {
                    startDistance = robot.getDistanceInchesLeft();
                    autonomousStep += 1;
                }
                break;
            case 2:
                correction = PIDSteering.calculate(robot.getYaw() - startAngle);
                leftPower = defaultPower * (1 + correction);
                rightPower = defaultPower * (1 - correction);
                if (! robot.getEscalatorSensorMedium()) {
                    differenceDistance = Math.abs(robot.getDistanceInchesLeft() - startDistance);
                    theta = Math.atan(differenceDistance / sensorDist)*180/Math.PI;
                    autonomousStep += 1;
                }
                break;
            case 3:
                leftPower = defaultPower*1.2;
                rightPower = defaultPower*0.8;
                currentYaw = robot.getYaw();
                if ( (currentYaw - startAngle <= -theta) ) {
                    autonomousStep += 1;
                }
                break;
            case 4:
                currentYaw = startAngle-theta;
                PIDSteering.reset();
                PIDSteering.enableContinuousInput(-180, 180);
                startDistance = robot.getDistanceInchesLeft();
                autonomousStep += 1;
                break;
            case 5:
                correction = PIDSteering.calculate(robot.getYaw() - currentYaw);
                leftPower = defaultPower * (1 - correction);
                rightPower = defaultPower * (1 + correction);
                if (Math.abs(robot.getDistanceInchesLeft()-startDistance)>12) {
                    leftPower = 0;
                    rightPower = 0;
                    autonomousStep += 1;
                }
                break;
            case 6: //adios amigos
                leftPower = 0;
                rightPower = 0;
                break;
        }
        robot.setMotorPowerPercentage(leftPower,rightPower);

    }


}