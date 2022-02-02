package frc.robot.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.GenericRobot;

import java.util.stream.DoubleStream;

public class wallToTape extends GenericAutonomous {

    double x_one = 0;
    double x_two = 0;

    double theta;
    double LidarGap = 9.25;

    double GoldOut = 26;
    double GoldIn = 22;

    boolean start = true;

    double defaultPower = .2;
    double rightPower;
    double leftPower;

    double correction;

    int count = 0;

    double xOneAr[] = new double[5];
    double xTwoAr[] = new double[5];

    double startAngle;

    double startDistance;
    double differenceDistance;

    double sensorDist = 12.0;
    double currentYaw;

    long startingTime = 0;

    boolean leftSensor = false;
    boolean rightSensor = false;

    double outerDistArc;
    double lTraveled;

    double fwd = 60;


    public void autonomousInit(GenericRobot robot) {
        start = true;
        count = 0;
        startingTime = System.currentTimeMillis();
        autonomousStep = -1;
        leftSensor = false;
        rightSensor = false;
        lTraveled = 0;
        fwd = 60;

    }

    public void autonomousPeriodic(GenericRobot robot) {
        PIDController PIDSteering = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
        SmartDashboard.putNumber("theta", theta);
        SmartDashboard.putNumber("xOne", x_one);
        SmartDashboard.putNumber("xTwo", x_two);
        SmartDashboard.putBoolean("High", robot.getEscalatorSensorHigh());
        SmartDashboard.putBoolean("Medium", robot.getEscalatorSensorMedium());
        SmartDashboard.putNumber("theta", theta);
        SmartDashboard.putNumber("l", lTraveled);
        SmartDashboard.putNumber("outerArcDist", outerDistArc);

        if (count < 5) {
            xOneAr[count] = robot.getLidarDistanceInchesFront()+1;
            xTwoAr[count] = robot.getLidarDistanceInchesLeft();
        } else {
            xOneAr[count % 5] = robot.getLidarDistanceInchesFront()+1;
            xTwoAr[count % 5] = robot.getLidarDistanceInchesLeft();
            x_one = DoubleStream.of(xOneAr).sum() / 5;
            x_two = DoubleStream.of(xTwoAr).sum() / 5;
        }
        count += 1;


        double x_avg = (x_one + x_two) / 2 * Math.cos(theta);

        switch (autonomousStep) {
            case -1:
                robot.resetEncoders();
                robot.resetAttitude();

                if (System.currentTimeMillis() >= startingTime + 100) {
                    autonomousStep += 1;
                }
                break;
            case 0:
                PIDSteering.reset();
                PIDSteering.enableContinuousInput(-180, 180);
                if (count > 5) {
                    autonomousStep += 1;
                }
                break;
            case 1:
                theta = Math.toDegrees(Math.atan((x_one-x_two)/ LidarGap));
                if (x_avg < GoldOut && x_avg > GoldIn) {
                    correction = PIDSteering.calculate(theta);
                    leftPower = defaultPower - correction;
                    rightPower = defaultPower + correction;
                }


                else if (x_avg > GoldOut) {
                    correction = PIDSteering.calculate(theta + 2.5);
                    leftPower = defaultPower - correction;
                    rightPower = defaultPower + correction;
                }

                else{
                    correction = PIDSteering.calculate(theta -2.5);
                    leftPower = defaultPower - correction;
                    rightPower = defaultPower + correction;
                }

                if (! robot.getEscalatorSensorHigh()) {
                    startAngle = robot.getYaw();
                    startDistance = robot.getDistanceInchesLeft();
                    leftSensor = true;
                    autonomousStep += 1;
                }
                else if (! robot.getEscalatorSensorMedium()) {
                    startAngle = robot.getYaw();
                    startDistance = robot.getDistanceInchesLeft();
                    rightSensor = true;
                    autonomousStep += 1;
                }
                break;
            case 2:
                correction = PIDSteering.calculate(robot.getYaw() - startAngle);
                leftPower = defaultPower + correction;
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
                    autonomousStep += 1;
                }
                break;
            case 3:
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
            case 4:
                correction = PIDSteering.calculate(robot.getYaw() - currentYaw);
                leftPower = defaultPower + correction;
                rightPower = defaultPower - correction;
                if (Math.abs(robot.getDistanceInchesLeft()-startDistance) >= (fwd-lTraveled)) {
                    leftPower = 0;
                    rightPower = 0;
                    autonomousStep += 1;
                }
                break;
            case 5: //adios amigos
                leftPower = 0;
                rightPower = 0;
                break;
        }
        robot.setMotorPowerPercentage(leftPower,rightPower);
        }

    }




