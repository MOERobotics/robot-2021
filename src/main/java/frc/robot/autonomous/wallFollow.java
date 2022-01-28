package frc.robot.autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.GenericRobot;

public class wallFollow extends GenericAutonomous{

    double x_one;
    double x_two;

    double theta;
    double LidarGap = 9.25;

    double GoldOut = 14;
    double GoldIn = 10;

    boolean start = true;

    double defaultPower = .2;
    double rightPower;
    double leftPower;

    double correction;





    public void autonomousInit(GenericRobot robot){
        start = true;

    }

    public void autonomousPeriodic(GenericRobot robot){
        PIDController PIDSteering = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
        SmartDashboard.putNumber("theta", theta);

        x_one = robot.getLidarDistanceInchesFront();
        x_two = robot.getLidarDistanceInchesLeft();

        double x_avg = (x_one+x_two)/2;

        theta = Math.toDegrees(Math.atan((x_two-x_one)/ LidarGap));

        if (start){
            PIDSteering.reset();
            PIDSteering.enableContinuousInput(-180, 180);
            start = false;
        }

        if (x_avg < GoldOut && x_avg > GoldIn){
            correction = PIDSteering.calculate(theta);
            leftPower = defaultPower + correction;
            rightPower = defaultPower - correction;
        }

        if (x_avg > GoldOut){
            correction = PIDSteering.calculate(theta-20);
            leftPower = defaultPower + correction;
            rightPower = defaultPower - correction;
        }

        if (x_avg < GoldIn){
            correction = PIDSteering.calculate(theta+20);
            leftPower = defaultPower + correction;
            rightPower = defaultPower - correction;
        }
        robot.setMotorPowerPercentage(leftPower,rightPower);
    }
}
