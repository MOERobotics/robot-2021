package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.GenericRobot;

public class TrenchRun{
    double defaultSpeed = 0.2;
    private boolean enabled = false;

    static double startingYaw      = 0.0;
    static double startingDistance = 0.0;
    //PIDModule PIDSteering = new PIDModule(4.0e-2, 0.0e-3, 1.0e-4);
    PIDController PIDSteering = new PIDController(4.0e-2, 0.0e-3, 1.0e-4);
    double correction;
    static double currentYaw = 0;
    double currentDistance = 0;
    double upperLimit = 14.2;
    double lowerLimit = 13.8;
    double newSpeed;

    public TrenchRun(double newSpeed){
        this.newSpeed = newSpeed;
    }

    public void begin(GenericRobot robot){
        PIDSteering.reset();
        PIDSteering.enableContinuousInput(-180,180);
    }

    public void run(GenericRobot robot, double newSpeed){

        defaultSpeed = newSpeed;
        if(robot.getLidarDistanceInchesLeft() != -9999){

            if((robot.getLidarDistanceInchesLeft() < upperLimit) && (robot.getLidarDistanceInchesLeft() > lowerLimit)){
                currentYaw = 0;
            }
            if(robot.getLidarDistanceInchesLeft() < lowerLimit){
                currentYaw = 6;
            }
            if(robot.getLidarDistanceInchesLeft() > upperLimit){
                currentYaw = -6;
            }
        }
        correction = PIDSteering.calculate(robot.getYaw() - currentYaw);
        robot.setMotorPowerPercentage(1.5 * defaultSpeed * (1 + correction), 1.5 * defaultSpeed * (1 - correction));
        currentDistance = robot.getDistanceInchesLeft();
    }


    public void setEnabled(boolean yesNo){
        this.enabled = yesNo;
    }

    public boolean getEnabled(){
        return this.enabled;
    }

}