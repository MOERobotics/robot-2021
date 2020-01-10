package frc.robot.genericrobot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.*;

public class CaMOElot extends GenericRobot {

    private TalonSRX leftMotorA = new TalonSRX(12);
    private TalonSRX leftMotorB = new TalonSRX(13);
    private TalonSRX leftMotorC = new TalonSRX(14);

    private TalonSRX rightMotorA = new TalonSRX(1);
    private TalonSRX rightMotorB = new TalonSRX(2);
    private TalonSRX rightMotorC = new TalonSRX(3);

    private Encoder leftEncoder = new Encoder(0,1);
    private Encoder rightEncoder = new Encoder(0,1);

    @Override
    public void setMotorPowerPercentage(double leftPower, double rightPower) {
        leftMotorA.set(ControlMode.PercentOutput, leftPower);
        leftMotorB.set(ControlMode.PercentOutput, leftPower);
        leftMotorC.set(ControlMode.PercentOutput, leftPower);
        rightMotorA.set(ControlMode.PercentOutput, rightPower);
        rightMotorB.set(ControlMode.PercentOutput, rightPower);
        rightMotorC.set(ControlMode.PercentOutput, rightPower);
    }

    @Override
    public double getDistanceInchesLeft() {
        return leftEncoder.get();
    }

    @Override
    public double getDistanceInchesRight() {
        return rightEncoder.get();
    }
}
