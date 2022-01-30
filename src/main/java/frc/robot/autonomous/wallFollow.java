package frc.robot.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.GenericRobot;

import java.util.stream.DoubleStream;

public class wallFollow extends GenericAutonomous {

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


    public void autonomousInit(GenericRobot robot) {
        start = true;
        count = 0;

    }

    public void autonomousPeriodic(GenericRobot robot) {
        PIDController PIDSteering = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
        /*SmartDashboard.putNumber("theta", theta);
        SmartDashboard.putNumber("xOne", x_one);
        SmartDashboard.putNumber("xTwo", x_two);
*/

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

        theta = Math.toDegrees(Math.atan((x_one-x_two)/ LidarGap));

        double x_avg = (x_one + x_two) / 2 * Math.cos(theta);


        if (start) {
            PIDSteering.reset();
            PIDSteering.enableContinuousInput(-180, 180);
            if (count > 5) {
                start = false;
            }
        } else {
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

            else/* (x_avg < GoldIn)*/ {
                correction = PIDSteering.calculate(theta -2.5);
                leftPower = defaultPower - correction;
                rightPower = defaultPower + correction;
            }
            robot.setMotorPowerPercentage(leftPower, rightPower);
        }

}



}
