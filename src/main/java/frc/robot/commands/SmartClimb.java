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
    double rollTrim;
    private boolean holding = false;

    PIDController PIDPortHold = new PIDController(1.0e-2, 2.0e-3, 0.0e-4);
    PIDController PIDStarboardHold = new PIDController(1.0e-2, 2.0e-3, 0.0e-4);
    PIDController PIDRollHold = new PIDController(1.0e-2, 2.0e-3, 0.0e-4);

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

            if (-robot.getPitch()> rollTolerance)
            {
                climbPowerStarboard += 0.2;
            }
            if (-robot.getPitch() < -rollTolerance)
            {
                climbPowerPort += 0.2;
            }

            encoderPort = robot.getClimberPortTicks();
            encoderStarboard = robot.getClimberStarboardTicks();
	        rollTrim = -robot.getPitch();
            PIDPortHold.reset();
            PIDStarboardHold.reset();
	        PIDRollHold.reset();
        }

        robot.setClimbVerticalPortPower     (-climbPowerPort);
        robot.setClimbVerticalStarboardPower(-climbPowerStarboard);
    }

    public void hold(GenericRobot robot){
        rollTrim = 0;

        double correctionPort = PIDPortHold.calculate(robot.getClimberPortTicks() - encoderPort);

        double correctionStarboard = PIDStarboardHold.calculate(robot.getClimberStarboardTicks() - encoderStarboard);

        double rollCorrection = PIDRollHold.calculate((-robot.getPitch()) - rollTrim);

        SmartDashboard.putNumber("errorPort", robot.getClimberPortTicks() - encoderPort);
        SmartDashboard.putNumber("correctionPort", correctionPort);
        SmartDashboard.putNumber("errorStarboard", robot.getClimberStarboardTicks() - encoderStarboard);
        SmartDashboard.putNumber("correctionStarboard", correctionStarboard);
        SmartDashboard.putNumber("errorRoll", -robot.getPitch() - rollTrim);
        SmartDashboard.putNumber("rollCorrection", rollCorrection);

	/*
	  rollTrim, encoderPort and encoderStarboard represent the state that we are trying to hold.
	  It's possible the bar will shift after we start to hold.

	  If you are listing to starboard, lower the port side and hold the starboard side.
	  If you are listing to port, lower the stb side and hold the port side.
	 */

        if (Math.abs(-robot.getPitch() - rollTrim)>rollTolerance) {
            if (-robot.getPitch() > rollTrim) {
                robot.setClimbVerticalPortPower(-rollCorrection);
                robot.setClimbVerticalStarboardPower(correctionStarboard);
            } else if (-robot.getPitch() < rollTrim) {
                robot.setClimbVerticalStarboardPower(rollCorrection);
                robot.setClimbVerticalPortPower(correctionPort);
            }
        } else {
            robot.setClimbVerticalStarboardPower(correctionStarboard);
            robot.setClimbVerticalPortPower(correctionPort);
        }

    }
}

