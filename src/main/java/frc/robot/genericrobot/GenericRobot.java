/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.genericrobot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;

public abstract class GenericRobot {

    public double leftPower = 0;
    public double rightPower = 0;
    public double shootUpperPower = 0;
    public double shootLowerPower = 0;
    public double spinPower = 0;

    public abstract void setMotorPowerPercentage(double leftPower, double rightPower);

    public void driveForward(double power){
        this.leftPower = leftPower;
        this.rightPower = rightPower;
        setMotorPowerPercentage(power, power);
    }

    public void driveBackward(double power){
        this.leftPower = -power;
        this.rightPower = -power;
        setMotorPowerPercentage(-power,-power);
    }

    public void driveLeftInPlace(double power){
        this.leftPower = -power;
        this.rightPower = power;
        setMotorPowerPercentage(-power,power);
    }

    public void driveRightInPlace(double power){
        this.leftPower = power;
        this.rightPower = -power;
        setMotorPowerPercentage(power,-power);
    }

    public double getDistanceInchesLeft(){
        return getDistanceRatioLeft() * getDistanceTicksLeft();
    }

    public double getDistanceInchesRight(){
        return getDistanceRatioRight() * getDistanceTicksRight();
    }

    public double getDistanceRatioLeft(){
        System.out.println("I don't have an encoder ;-;");
        return 0;
    }

    public double getDistanceRatioRight(){
        System.out.println("I don't have an encoder ;-;");
        return 0;
    }

    public double getDistanceTicksLeft(){
        System.out.println("I don't have an encoder ;-;");
        return 0;
    }

    public double getDistanceTicksRight(){
        System.out.println("I don't have an encoder ;-;");
        return 0;
    }

    public double getMotorPowerLeft(){
        return leftPower;
    }

    public double getMotorPowerRight(){
        return rightPower;
    }

    public double getYaw(){
        System.out.println("I don't have a navx ;-;");
        return 0.0;
    }

    public double getPicth(){
        System.out.println("I don't have a navx ;-;");
        return 0.0;
    }

    public double getRoll(){
        System.out.println("I don't have a navx ;-;");
        return 0.0;
    }

    public void setShooterPowerPercentage(double shootUpperPower, double shootLowerPower){
        this.shootUpperPower = shootUpperPower;
        this.shootLowerPower = shootLowerPower;
        setShooterPowerPercentageInternal(shootUpperPower, shootLowerPower);
    }

    public void setShooterPowerPercentageInternal(double upperPower, double lowerPower){
        System.out.println("I don't have a shooter ;-;");
    }

    public double getShooterPowerUpper(){
        return shootUpperPower;
    }

    public double getShooterPowerLower(){
        return shootLowerPower;
    }

    public void spinControlPanel(double power){
        spinPower = power;
        spinControlPanelInternal(power);
    }

    protected void spinControlPanelInternal(double power){
        System.out.println("I can't spin the control panel ;-;");
    }

    public double getControlPanelSpinnerPower(){
        return spinPower;
    }

    public char getCurrentWheelColor(){
        System.out.println("I don't have a color sensor ;-;");
        return '?';
    }
}
