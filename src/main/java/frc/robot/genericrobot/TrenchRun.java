package frc.robot.genericrobot;

import frc.robot.PIDModule;

public class TrenchRun {
    double defaultSpeed = 0.2;

    static double startingYaw      = 0.0;
    static double startingDistance = 0.0;
    PIDModule PIDSteering = new PIDModule(4.0e-2, 0.0e-3, 1.0e-4);
    double correction;
    static double currentYaw = 0;
    double currentDistance = 0;


    public void init(){
        PIDSteering.resetError();
    }

    public void run(GenericRobot robot){
        if((robot.getLidarDistanceLeft() < 630) && (robot.getLidarDistanceLeft() > 570)){
            currentYaw = 0;
        }
        if(robot.getLidarDistanceLeft() < 550){
            currentYaw = 5;
        }
        if(robot.getLidarDistanceLeft() > 650){
            currentYaw = -5;
        }
        PIDSteering.sendError(robot.getYaw() - currentYaw);
        correction = PIDSteering.getCorrection();
        robot.setMotorPowerPercentage(1.5 * defaultSpeed * (1 + correction), 1.5 * defaultSpeed * (1 - correction));
        currentDistance = robot.getDistanceInchesLeft();
    }
}
