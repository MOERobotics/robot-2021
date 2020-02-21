package frc.robot.commands;

import frc.robot.genericrobot.GenericRobot;

public class LimelightAlign extends GenericCommand{
    double leftPower;
    double rightPower;
    private boolean aligning;
    double currentDistance = 0;
    double setPoint;
    double setPointDeadzone;
    double constant;
    boolean targetCentered;
    long startingTime = 0;

    public LimelightAlign(double setPoint, double setPointDeadzone, double constant){

        this.setPoint = setPoint;
        this.setPointDeadzone = setPointDeadzone;
        this.constant = constant;

    }
    @Override
    public void begin(GenericRobot robot) {
        targetCentered = false;
    }

    @Override
    public void step(GenericRobot robot) {
        double minPower = .08;
        double currentTime = System.currentTimeMillis();

        aligning = true;

        if (robot.limelight.getLimelightX() < -setPointDeadzone + -setPoint) {
            //Pivots to the left
            targetCentered = false;
            currentDistance = Math.abs(robot.limelight.getLimelightX() + setPoint);
            leftPower = -(constant * currentDistance);
            rightPower = constant * currentDistance;
            if (rightPower <= minPower) {
                leftPower = -minPower;
                rightPower = minPower;
            }

        } else if (robot.limelight.getLimelightX() > setPointDeadzone + setPoint) {
            //Pivots to the right
            targetCentered = false;
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
            if(!targetCentered){
                startingTime = System.currentTimeMillis();
                targetCentered = true;
            }


            if(targetCentered && currentTime - startingTime > 600){
                aligning = false;
            }

        }

        if(!aligning){
            setEnabled(false);
        }

        robot.setMotorPowerPercentage(leftPower, rightPower);

    }


}
