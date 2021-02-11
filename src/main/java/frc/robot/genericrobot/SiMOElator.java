package frc.robot.genericrobot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Util;

import static java.lang.Math.*;

public class SiMOElator extends GenericRobot {
	double x_meters, y_meters, theta_rad;
	double leftDistance, rightDistance;
	double linearVelocity;
	double angularVelocity;
	DriverStation robotControlCenter;
	Field2d drawing = new Field2d();

	public static final double
		NORTH =  0,
		EAST  =  PI/2.0,
		SOUTH =  PI,
		WEST  = -PI/2.0;


	public SiMOElator(DriverStation robotControlCenter) {
		this.robotControlCenter = robotControlCenter;
		resetRobotSimulation();
	}

	@Override protected void updateRobotInternal() {
		if (robotControlCenter.isDisabled()) return;

		double current_x     = x_meters;
		double current_y     = y_meters;
		double current_theta = theta_rad;
		double current_linearVelocity  =  linearVelocity;
		double current_angularVelocity = angularVelocity;

		double delta_x     = sin(current_theta)*current_linearVelocity * 0.06;
		double delta_y     = cos(current_theta)*current_linearVelocity * 0.06;
		double delta_theta = current_angularVelocity * 0.06;

		double new_x     = current_x + delta_x;
		double new_y     = current_y + delta_y;
		double new_theta = Util.normalizeAngleRadians(current_theta+delta_theta);;

		 leftDistance += (current_linearVelocity + current_angularVelocity)*0.06;
		rightDistance += (current_linearVelocity - current_angularVelocity)*0.06;

		x_meters = new_x;
		y_meters = new_y;
		theta_rad = new_theta;

		drawRobot();
	}

	@Override
	protected void printSmartDashboardInternal() {
		SmartDashboard.putNumber("Simulator X" , x_meters);
		SmartDashboard.putNumber("Simulator Y" , y_meters);
		SmartDashboard.putNumber("Simulator T" , theta_rad);
		SmartDashboard.putNumber("Simulator LV", linearVelocity);
		SmartDashboard.putNumber("Simulator AV", angularVelocity);
	}

	@Override public void resetRobotSimulation() {
		//Center of field
		x_meters  = 1;
		y_meters  = 1;
		theta_rad = EAST;

		 linearVelocity=0;
		angularVelocity=0;
		resetEncoderLeft();
		resetEncoderRight();
		drawRobot();
	}
	private void drawRobot() {
		drawing.setRobotPose(
			x_meters,
			y_meters,
			new Rotation2d(EAST - theta_rad)
		);
	}

	private double coolDeadzone(double input, double deadzone) {
		if (input >  deadzone) return (input - deadzone)/(1-deadzone);
		if (input < -deadzone) return (input + deadzone)/(1-deadzone);
		return 0;
	}

	@Override
	protected void setMotorPowerPercentageInternal(double leftPower, double rightPower) {
		double linearComponent = (leftPower + rightPower) / 2.0;
		double angularComponent = (linearComponent - rightPower)*2*12/28;

		//linearComponent = coolDeadzone( linearComponent, 0.15);
		//angularComponent = coolDeadzone(angularComponent, 0.10);

		linearVelocity = linearComponent;
		angularVelocity = angularComponent;
	}

	@Override public double getYaw() {
		return toDegrees(theta_rad);
	}

	@Override
	public double getDistanceRatioLeft() {
		return 1.0/12.0;
	}
	@Override
	public double getDistanceRatioRight() { return 1.0/12.0; }

	@Override
	public double getDistanceTicksLeft() {
		return leftDistance;
	}


	@Override
	public double getDistanceTicksRight() {
		return rightDistance;
	}

	@Override
	public void resetEncoderLeft() {
		leftDistance = 0;
	}

	@Override
	public void resetEncoderRight() {
		rightDistance = 0;
	}
}
