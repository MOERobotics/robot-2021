package frc.robot.vision;


import frc.robot.genericrobot.GenericRobot;

public class AutoAlign {

    double leftPower;
    double rightPower;
    boolean aligned;

    public void run(GenericRobot robot, double setPoint, double setPointDeadzone) {
        double currentDistance = 0;
        double minPower = .04;
        if (robot.getLimelightX() < -setPointDeadzone + -setPoint) {
            //Pivots to the left
            currentDistance = Math.abs(robot.getLimelightX() + setPoint);
            leftPower = -(.01852 * currentDistance);
            rightPower = .01852 * currentDistance;
            if (rightPower <= minPower) {
                leftPower = -minPower;
                rightPower = minPower;
            }

        } else if (robot.getLimelightX() > setPointDeadzone + setPoint) {
            //Pivots to the right
            currentDistance = Math.abs(robot.getLimelightX() - setPoint);
            leftPower = .01852 * currentDistance;
            rightPower = -(.01852 * currentDistance);
            if (leftPower <= minPower) {
                leftPower = minPower;
                rightPower = -minPower;
            }

        } else {
            leftPower = 0;
            rightPower = 0;
        }

        robot.setMotorPowerPercentage(leftPower, rightPower);
    }
}
