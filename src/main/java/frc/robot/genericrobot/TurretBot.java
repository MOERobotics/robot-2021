package frc.robot.genericrobot;

import com.revrobotics.CANSparkMax;

import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Logger;

import java.beans.Encoder;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

public class TurretBot extends GenericRobot {

    CANSparkMax indexer   = new CANSparkMax(40, kBrushed);
    CANSparkMax speeeen   = new CANSparkMax(41, kBrushless);
    CANSparkMax shooterA  = new CANSparkMax(42, kBrushless);
    CANSparkMax shooterB  = new CANSparkMax(49, kBrushless);
    CANSparkMax collector = new CANSparkMax(43, kBrushed);
    CANSparkMax leftMotorA = new CANSparkMax(20, kBrushless);
    CANSparkMax leftMotorB = new CANSparkMax(1, kBrushless);
    CANSparkMax rightMotorA = new CANSparkMax(14, kBrushless);
    CANSparkMax rightMotorB = new CANSparkMax(15, kBrushless);
    Solenoid    shifter = new Solenoid(0);

    @Override
    protected void setMotorPowerPercentageInternal(double leftPower, double rightPower) {
        //super.setMotorPowerPercentageInternal(leftPower,rightPower);

        leftMotorA.set(leftPower);
        leftMotorB.set(leftPower);
        rightMotorA.set(rightPower);
        rightMotorB.set(rightPower);
    }

    @Override
    protected void setShooterPowerPercentageInternal(double upperPower, double lowerPower) {
        super.setShooterPowerPercentageInternal(upperPower, lowerPower);
        shooterB.follow(shooterA,true);
        shooterA.set(upperPower);
        //shooterB.set(lowerPower);
    }

    @Override
    protected void setShooterRPMInternal(double upperRPM, double lowerRPM) {
        super.setShooterPowerPercentageInternal(upperRPM, lowerRPM);
        shooterB.follow(shooterA,true);
        shooterA.getPIDController().setReference(upperRPM, ControlType.kVelocity);
        //shooterB.getPIDController().setReference(lowerRPM, ControlType.kVelocity);
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
            SHOOTER_SPEED_OFF = new ShooterSpeedPreset(0, 0),
            SHOOTER_SPEED_SHORT = new ShooterSpeedPreset(2285, 2285),
            SHOOTER_SPEED_MID = new ShooterSpeedPreset(2620, 2620),
            SHOOTER_SPEED_LONG = new ShooterSpeedPreset(4000, 3000), //not final
            SHOOTER_SPEED_YEET = new ShooterSpeedPreset(5000, 5000);


    @Override
    public ShooterSpeedPreset getShooterSpeedPreset(
            ShooterSpeedPresetName speedType

    ) {
        switch (speedType) {
            case SHORT_RANGE:
                return SHOOTER_SPEED_SHORT;
            case MID_RANGE:
                return SHOOTER_SPEED_MID;
            case LONG_RANGE:
                return SHOOTER_SPEED_LONG;
            case YEET:
                return SHOOTER_SPEED_YEET;
            default:
                return SHOOTER_SPEED_OFF;

        }
    }

    @Override
    public double getShooterVelocityRPMUpper() {
        return shooterA.getEncoder().getVelocity();
    }

    @Override
    public double getShooterVelocityRPMLower() {
        return shooterB.getEncoder().getVelocity();
    }


    @Override
    public void setTurretPowerPercentageInternal(double power) {
        speeeen.set(power);
    }

    @Override
    public double getTurretAngleDegrees() {
        return speeeen.getEncoder().getPosition();
    }
}
