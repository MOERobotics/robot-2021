package frc.robot.commands;

import frc.robot.genericrobot.GenericRobot;

public class CollectPowerCells extends GenericCommand{
    double leftPower;
    double rightPower;
    private boolean aligning;
    double currentDistance = 0;
    double setPoint;
    double setPointDeadzone;
    double constant;

    public CollectPowerCells(double setPoint, double setPointDeadzone, double constant){

        this.setPoint = setPoint;
        this.setPointDeadzone = setPointDeadzone;
        this.constant = constant;

    }
    @Override
    public void begin(GenericRobot robot) {

    }

    @Override
    public void step(GenericRobot robot) {
        double minPower = .04;

        aligning = true;

        if (robot.limelight.getLimelightX() < -setPointDeadzone + -setPoint) {
            //Pivots to the left
            currentDistance = Math.abs(robot.limelight.getLimelightX() + setPoint);
            leftPower = -(constant * currentDistance);
            rightPower = constant * currentDistance;
            if (rightPower <= minPower) {
                leftPower = -minPower;
                rightPower = minPower;
            }

        } else if (robot.limelight.getLimelightX() > setPointDeadzone + setPoint) {
            //Pivots to the right
            currentDistance = Math.abs(robot.limelight.getLimelightX() - setPoint);
            leftPower = constant * currentDistance;
            rightPower = -(constant * currentDistance);
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
                    600
            );

        }

        if(!aligning){
            setEnabled(false);
        }

        robot.setMotorPowerPercentage(leftPower, rightPower);

    }


}
