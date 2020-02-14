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
    boolean dunzo;
    long startingTime = 0;

    public LimelightAlign(double setPoint, double setPointDeadzone, double constant){

        this.setPoint = setPoint;
        this.setPointDeadzone = setPointDeadzone;
        this.constant = constant;

    }
    @Override
    public void begin(GenericRobot robot) {
        dunzo = false;
    }

    @Override
    public void step(GenericRobot robot) {
        double minPower = .04;
        double currentTime = System.currentTimeMillis();

        aligning = true;

        if (robot.limelight.getLimelightX() < -setPointDeadzone + -setPoint) {
            //Pivots to the left
            dunzo = false;
            currentDistance = Math.abs(robot.limelight.getLimelightX() + setPoint);
            leftPower = -(constant * currentDistance);
            rightPower = constant * currentDistance;
            if (rightPower <= minPower) {
                leftPower = -minPower;
                rightPower = minPower;
            }

        } else if (robot.limelight.getLimelightX() > setPointDeadzone + setPoint) {
            //Pivots to the right
            dunzo = false;
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
            if(!dunzo){
                startingTime = System.currentTimeMillis();
                dunzo = true;
            }


            if(dunzo && currentTime - startingTime > 600){
                aligning = false;
            }

        }

        if(!aligning){
            setEnabled(false);
        }

        robot.setMotorPowerPercentage(leftPower, rightPower);

    }


}
