package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.GenericRobot;

public class LimelightAlign extends GenericCommand{
    double leftPower;
    double rightPower;
    private boolean aligning;
    double setPoint;
    double setPointDeadzone;
    boolean targetCentered;
    long startingTime = 0;
    long startTime;
    long timeoutTime = 4000;
    double correction;
    double defaultSpeed = 0.35;
    PIDController PIDPivot = new PIDController(6.0e-2, 1.0e-2, 1.0e-3);

    public LimelightAlign(double setPoint, double setPointDeadzone){

        this.setPoint = -setPoint;
        this.setPointDeadzone = setPointDeadzone;
        PIDPivot.reset();

    }
    @Override
    public void begin(GenericRobot robot) {
        targetCentered = false;
        startTime = System.currentTimeMillis();
        PIDPivot.reset();
    }

    @Override
    public void step(GenericRobot robot) {

        long currentTime = System.currentTimeMillis();

        aligning = true;

        /*
        if((currentTime - startTime) > timeoutTime) {
            System.out.println("Limelight timed out.");
            aligning = false;
        }
        */

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

        SmartDashboard.putBoolean("targeted", !aligning);
        if(!aligning){
            setEnabled(false);
        }

        robot.setMotorPowerPercentage(leftPower, rightPower);

    }

    public void setSetPoint(double setPoint){
        this.setPoint = setPoint;
    }

    public double getSetPoint(){
        return setPoint;
    }


}
