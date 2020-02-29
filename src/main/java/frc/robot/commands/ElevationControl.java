package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.genericrobot.GenericRobot;

public class ElevationControl {
    private boolean enabled = false;
    double setPoint = 127; //yeet
    PIDController PIDElevation = new PIDController(1.0e-1, 1.0e-2, 1.0e-3);

    public ElevationControl(){
        PIDElevation.disableContinuousInput();
        PIDElevation.reset();
    }

    public void begin(GenericRobot robot){
        PIDElevation.reset();
    }

    public void run(GenericRobot robot){
        double correction = PIDElevation.calculate(robot.getElevation() - setPoint);
        correction = Math.max(correction, -0.5);
        correction = Math.min(correction, 0.5);
        robot.setAngleAdjusterPower(correction);
    }

    public void setEnabled(boolean yesNo){
        this.enabled = yesNo;
    }

    public boolean getEnabled(){
        return this.enabled;
    }

    public void setSetPoint(double setPointValue){
        this.setPoint = setPointValue;
    }


}
