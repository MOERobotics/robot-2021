package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.genericrobot.GenericRobot;

public class LimelightAlign extends GenericCommand{
    double leftPower;
    double rightPower;
    private boolean aligning;
    double setPoint;
    double setPointDeadzone;
    double constant;
    boolean targetCentered;
    long startingTime = 0;
    long startTime;
    long timeoutTime = 4000;
    double correction;
    double defaultSpeed = 0.35;

    public LimelightAlign(double setPoint, double setPointDeadzone, double constant){

        this.setPoint = -setPoint;
        this.setPointDeadzone = setPointDeadzone;
        this.constant = constant;

    }
    @Override
    public void begin(GenericRobot robot) {
        targetCentered = false;
        startTime = System.currentTimeMillis();
    }

    @Override
    public void step(GenericRobot robot) {
        PIDController PIDPivot = new PIDController(4.5e-2, 3.0e-2, 1.0e-4);

        long currentTime = System.currentTimeMillis();

        aligning = true;

        if((currentTime - startTime) > timeoutTime){
            aligning = false;
        }

        if (robot.limelight.getLimelightX() < -setPointDeadzone + setPoint) {
            //Pivots to the left
            targetCentered = false;

            correction = PIDPivot.calculate(setPoint - robot.limelight.getLimelightX());
            leftPower = defaultSpeed * correction;
            rightPower = -defaultSpeed * correction;


        } else if (robot.limelight.getLimelightX() > setPointDeadzone + setPoint) {
            //Pivots to the right
            targetCentered = false;
            correction = PIDPivot.calculate(robot.limelight.getLimelightX() - setPoint);
            leftPower = -defaultSpeed * correction;
            rightPower = defaultSpeed * correction;

        } else {
            leftPower = 0;
            rightPower = 0;
            if (!targetCentered) {
                startingTime = System.currentTimeMillis();
                targetCentered = true;
            }

            if(targetCentered && currentTime - startingTime > 500){
                aligning = false;
            }

        }

        if(!aligning){
            setEnabled(false);
        }

        robot.setMotorPowerPercentage(leftPower, rightPower);

    }


}
