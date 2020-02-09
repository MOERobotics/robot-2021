package frc.robot.commands;

import frc.robot.PIDModule;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.commands.GenericCommand;
import frc.robot.genericrobot.GenericRobot;

public class TrenchRun extends GenericCommand {
    double defaultSpeed = 0.2;

    static double startingYaw      = 0.0;
    static double startingDistance = 0.0;
    //PIDModule PIDSteering = new PIDModule(4.0e-2, 0.0e-3, 1.0e-4);
    PIDController PIDSteering = new PIDController(4.0e-2, 0.0e-3, 1.0e-4);
    double correction;
    static double currentYaw = 0;
    double currentDistance = 0;
    Integer upperLimit = 630;
    Integer lowerLimit = 570;
    double newSpeed;

    public TrenchRun(double newSpeed){
        this.newSpeed = newSpeed;
    }

    @Override
    public void begin(GenericRobot robot){
        setControlLock(false);
        PIDSteering.reset();
        PIDSteering.enableContinuousInput(-180,180);
    }

    @Override
    public void step(GenericRobot robot){
        run(robot, newSpeed);
    }

    public void run(GenericRobot robot, double newSpeed){
        // Rear lidar values
        // 3 inches = 119 | 5 inches = 165 | 10 inches = 302 | 15 inches = 434 | 20 inches = 561
        defaultSpeed = newSpeed;
        if(robot.getLidarDistanceLeft() != -9999){
            if((robot.getLidarDistanceLeft() < upperLimit) && (robot.getLidarDistanceLeft() > lowerLimit)){
                currentYaw = 0;
            }
            if(robot.getLidarDistanceLeft() < lowerLimit){
                currentYaw = 6;
            }
            if(robot.getLidarDistanceLeft() > upperLimit){
                currentYaw = -6;
            }
        }
        correction = PIDSteering.calculate(robot.getYaw() - currentYaw);
        robot.setMotorPowerPercentage(1.5 * defaultSpeed * (1 + correction), 1.5 * defaultSpeed * (1 - correction));
        currentDistance = robot.getDistanceInchesLeft();
    }

}
