package frc.robot.genericrobot;

import frc.robot.PIDModule;
import edu.wpi.first.wpilibj.controller.PIDController;

public class TrenchRun {
    double defaultSpeed = 0.2;

    static double startingYaw      = 0.0;
    static double startingDistance = 0.0;
    //PIDModule PIDSteering = new PIDModule(4.0e-2, 0.0e-3, 1.0e-4);
    PIDController PIDSteering = new PIDController(4.0e-2, 0.0e-3, 1.0e-4);
    double correction;
    static double currentYaw = 0;
    double currentDistance = 0;


    public void init(){
        PIDSteering.reset();
        PIDSteering.enableContinuousInput(-180,180);
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
        correction = PIDSteering.calculate(robot.getYaw() - currentYaw);
        robot.setMotorPowerPercentage(1.5 * defaultSpeed * (1 + correction), 1.5 * defaultSpeed * (1 - correction));
        currentDistance = robot.getDistanceInchesLeft();
    }
}
