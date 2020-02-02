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

    public void run(GenericRobot robot, Lidar lidar){
        if(lidar.getDistance(3) < 650 && lidar.getDistance(3) > 550){
            currentYaw = 0;
        }
        PIDSteering.sendError(robot.getYaw() - currentYaw);
        correction = PIDSteering.getCorrection();
        robot.setMotorPowerPercentage(1.5 * defaultSpeed * (1 + correction), 1.5 * defaultSpeed * (1 - correction));
        currentDistance = robot.getDistanceInchesLeft();
    }
}
