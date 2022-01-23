package frc.robot.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.GenericRobot;

public class TapeAlign extends GenericAutonomous{

    double startAngle;

    double leftPower;
    double rightPower;
    double defaultPower = .2;

    double startDistance;
    double differenceDistance;

    double sensorDist = 12.0;
    double theta = 0;

    double correction;
    double currentYaw;

    long startingTime = 0;

    boolean leftSensor = false;
    boolean rightSensor = false;

    double outerDistArc;
    double lTraveled;

    double fwd = 60;

    public void autonomousInit(GenericRobot robot) {
        startingTime = System.currentTimeMillis();
        autonomousStep = -1;
        leftSensor = false;
        rightSensor = false;
        lTraveled = 0;
        fwd = 60;
    }

    public void autonomousPeriodic(GenericRobot robot){
        PIDController PIDSteering = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
        double yawError;
        SmartDashboard.putBoolean("High", robot.getEscalatorSensorHigh());
        SmartDashboard.putBoolean("Medium", robot.getEscalatorSensorMedium());
        SmartDashboard.putNumber("theta", theta);
        SmartDashboard.putNumber("l", lTraveled);
        SmartDashboard.putNumber("outerArcDist", outerDistArc);
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
                leftPower = defaultPower + correction; //didn't we stop doing this?
                rightPower = defaultPower - correction;

                if (! robot.getEscalatorSensorHigh()) {
                    startDistance = robot.getDistanceInchesLeft();
                    leftSensor = true;
                    autonomousStep += 1;
                }
                else if (! robot.getEscalatorSensorMedium()) {
                    startDistance = robot.getDistanceInchesLeft();
                    rightSensor = true;
                    autonomousStep += 1;
                }
                break;
            case 2:
                correction = PIDSteering.calculate(robot.getYaw() - startAngle);
                leftPower = defaultPower + correction; //confusion
                rightPower = defaultPower - correction;

                if ( !rightSensor && ! robot.getEscalatorSensorMedium()) {
                    differenceDistance = Math.abs(robot.getDistanceInchesLeft() - startDistance);
                    theta = Math.atan(differenceDistance / sensorDist)*180/Math.PI;
                    outerDistArc = robot.getDistanceInchesRight();
                    autonomousStep += 1;
                }
                else if (!leftSensor && ! robot.getEscalatorSensorHigh()){
                    differenceDistance = Math.abs(robot.getDistanceInchesLeft() - startDistance);
                    theta = Math.atan(differenceDistance / sensorDist)*180/Math.PI;
                    outerDistArc = robot.getDistanceInchesLeft();
                    autonomousStep = 4;
                }
                break;
            case 3://///////////////////////////////////////////////////////////////////skip this step
                if (leftSensor){
                    leftPower = defaultPower*.5;
                    rightPower = defaultPower*2;
                }
                else {
                    rightPower = defaultPower * .5;
                    leftPower = defaultPower * 2;
                }
                currentYaw = robot.getYaw();
                if ( Math.abs(Math.signum(currentYaw - startAngle)*(((Math.abs(currentYaw - startAngle) + 180) % 360) - 180)) >= Math.abs(theta) ) {
                    if (rightSensor){
                        outerDistArc = robot.getDistanceInchesLeft() - outerDistArc;
                    }
                    else {
                        outerDistArc = robot.getDistanceInchesRight() - outerDistArc;
                    }
                    lTraveled = Math.abs(outerDistArc/(theta*Math.PI/180)*Math.sin(Math.abs(theta*Math.PI/180)));
                    autonomousStep += 1;
                }///////////////////////////////////////this step is skipped
                break;
            case 4:
                if (leftSensor) {
                    currentYaw = startAngle - theta; //currentYaw = targetYaw because we are lazy
                }
                else{
                    currentYaw = startAngle + theta; //currentYaw = targetYaw because we are lazy
                }
                PIDSteering.reset();
                PIDSteering.enableContinuousInput(-180, 180);
                startDistance = robot.getDistanceInchesLeft();
                autonomousStep += 1;
                break;
            case 5:
                correction = PIDSteering.calculate(robot.getYaw() - currentYaw);
                leftPower = defaultPower + correction;
                rightPower = defaultPower - correction;
                if (Math.abs(robot.getDistanceInchesLeft()-startDistance) >= (fwd-lTraveled)) {
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