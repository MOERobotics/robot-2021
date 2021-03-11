package frc.robot.genericrobot;

import com.revrobotics.CANSparkMax;

import com.revrobotics.ControlType;

import frc.robot.Logger;

import java.beans.Encoder;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

public class TurretBot extends GenericRobot{

    @Override
    protected void setMotorPowerPercentageInternal(double leftPower, double rightPower) {
        Logger.logOnce("TurretBotDriveMotors",
                "Believe it or not, these fools didn't give me a drive motor");
    }

    CANSparkMax indexer = new CANSparkMax(40, kBrushed);
    CANSparkMax speeeen = new CANSparkMax(41, kBrushless);
    CANSparkMax shooterA = new CANSparkMax(42, kBrushless);;
    CANSparkMax shooterB = new CANSparkMax(49, kBrushless);;
    CANSparkMax collector = new CANSparkMax(43, kBrushed);



    @Override
    protected void setShooterPowerPercentageInternal(double upperPower, double lowerPower) {
        super.setShooterPowerPercentageInternal(upperPower, lowerPower);
        shooterA.set(upperPower);
        shooterB.set(lowerPower);


    }

    @Override
    protected void setShooterRPMInternal(double upperRPM, double lowerRPM) {
        shooterA.getPIDController().setReference(upperRPM, ControlType.kVelocity);
        shooterB.getPIDController().setReference(lowerRPM, ControlType.kVelocity);
    }
    @Override
    protected void setIndexerPowerInternal(double indexerPower) {
        indexer.set(indexerPower);
    }

    @Override
    protected void setCollectorPowerInternal(double collectorPower) {
        collector.set(-collectorPower);
    }

    private static final ShooterSpeedPreset
            SHOOTER_SPEED_OFF = new ShooterSpeedPreset(0,0),
            SHOOTER_SPEED_SHORT = new ShooterSpeedPreset(2285, 2285),
            SHOOTER_SPEED_MID = new ShooterSpeedPreset(2620, 2620),
            SHOOTER_SPEED_LONG = new ShooterSpeedPreset(4000, 3000), //not final
            SHOOTER_SPEED_YEET = new ShooterSpeedPreset(5000, 5000);




    @Override
    public ShooterSpeedPreset getShooterSpeedPreset(
            ShooterSpeedPresetName speedType

    )
    {
        switch (speedType){
            case SHORT_RANGE : return SHOOTER_SPEED_SHORT;
            case MID_RANGE   : return SHOOTER_SPEED_MID;
            case LONG_RANGE  : return SHOOTER_SPEED_LONG;
            case YEET        : return SHOOTER_SPEED_YEET;
            default          : return SHOOTER_SPEED_OFF;

        }
    }
    @Override
    public double getShooterVelocityRPMUpper(){
        return shooterA.getEncoder().getVelocity();
    }
    @Override
    public double getShooterVelocityRPMLower(){
        return shooterB.getEncoder().getVelocity();
    }







}