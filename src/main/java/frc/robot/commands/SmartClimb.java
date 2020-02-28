package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.GenericRobot;

public class SmartClimb{

    boolean engagedPort = false;
    boolean engagedStarboard = false;
    double climbPowerPort;
    double climbPowerStarboard;
    double rollTolerance=2.0;
    double currentTolerance = 5.0;
    long currentTime;
    long startTime;
    double encoderPort;
    double encoderStarboard;
    private boolean holding = false;

    PIDController PIDPortHold = new PIDController(1.0e-2, 2.0e-3, 0.0e-4);
    PIDController PIDStarboardHold = new PIDController(1.0e-2, 2.0e-3, 0.0e-4);

    public SmartClimb(){

    }

    public void setHolding(boolean input){
        holding = input;
    }

    public boolean getHolding(){
        return holding;
    }

    public void begin(GenericRobot robot) {
        engagedPort = false;
        engagedStarboard = false;
        holding = false;
        startTime = System.currentTimeMillis();
        PIDPortHold.disableContinuousInput();
        PIDStarboardHold.disableContinuousInput();
    }

    public void step(GenericRobot robot) {
        climbPowerPort = 0.2;
        climbPowerStarboard = 0.2;
        currentTime = System.currentTimeMillis();

        if (currentTime-startTime > 500) {
            if (Math.abs(robot.getClimberVerticalPortCurrent()) > currentTolerance) {
                engagedPort = true;
            }
            if (Math.abs(robot.getClimberVerticalStarboardCurrent()) > currentTolerance) {
                engagedStarboard = true;
            }
        }

        if (engagedPort)
        {
            climbPowerPort = 0.0;
        }

        if (engagedStarboard)
        {
            climbPowerStarboard = 0.0;
        }

        if (engagedPort && engagedStarboard)
        {
            climbPowerPort = 0.6;
            climbPowerStarboard = 0.6;

            if (robot.getRoll()> rollTolerance)
            {
                climbPowerStarboard += 0.2;
            }
            if (robot.getRoll() < -rollTolerance)
            {
                climbPowerPort += 0.2;
            }

            encoderPort = robot.getClimberPortTicks();
            encoderStarboard = robot.getClimberStarboardTicks();
            PIDPortHold.reset();
            PIDStarboardHold.reset();
        }

        robot.setClimbVerticalPortPower     (-climbPowerPort);
        robot.setClimbVerticalStarboardPower(-climbPowerStarboard);
    }

    public void hold(GenericRobot robot){
        double correctionPort = PIDPortHold.calculate(robot.getClimberPortTicks() - encoderPort);

        double correctionStarboard = PIDStarboardHold.calculate(robot.getClimberStarboardTicks() - encoderStarboard);

        SmartDashboard.putNumber("errorPort", robot.getClimberPortTicks() - encoderPort);
        SmartDashboard.putNumber("correctionPort", correctionPort);
        SmartDashboard.putNumber("errorStarboard", robot.getClimberStarboardTicks() - encoderStarboard);
        SmartDashboard.putNumber("correctionStarboard", correctionStarboard);

        robot.setClimbVerticalPortPower(correctionPort);
        robot.setClimbVerticalStarboardPower(correctionStarboard);
    }
}
