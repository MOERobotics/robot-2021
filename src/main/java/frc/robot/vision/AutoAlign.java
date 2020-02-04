package frc.robot.vision;


import frc.robot.genericrobot.GenericRobot;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Timer;

public class AutoAlign {

    double leftPower;
    double rightPower;
    private boolean aligning = true;
    double currentDistance = 0;

    public boolean run(GenericRobot robot, double setPoint, double setPointDeadzone, int holdTime) {
        double minPower = .04;

        //don't remove this
        aligning = true;

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

            new java.util.Timer().schedule(
                    new java.util.TimerTask() {
                        @Override
                        public void run() {
                            aligning = false;
                        }
                    },
                    holdTime
            );



        }

        robot.setMotorPowerPercentage(leftPower, rightPower);
        return aligning;

    }
}
