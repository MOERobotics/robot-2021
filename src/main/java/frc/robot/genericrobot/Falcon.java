package frc.robot.genericrobot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class Falcon extends GenericRobot{

    CANSparkMax leftDriveA = new CANSparkMax(13, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax leftDriveB = new CANSparkMax( 14, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax leftDriveC  = new CANSparkMax(15, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax rightDriveA  = new CANSparkMax(20, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax rightDriveB = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax rightDriveC = new CANSparkMax( 2, CANSparkMaxLowLevel.MotorType.kBrushless);

    CANSparkMax climberA = new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax climberB = new CANSparkMax( 3, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax generatorShift  = new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless);

    CANSparkMax shooterA  = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax shooterB  = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax indexer = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax conveyorBelt = new CANSparkMax( 7, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax angleAdj = new CANSparkMax(8, CANSparkMaxLowLevel.MotorType.kBrushless);

    CANSparkMax controlPanel = new CANSparkMax( 9, CANSparkMaxLowLevel.MotorType.kBrushless);

    CANSparkMax collector  = new CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless);

    public Falcon() {
        leftDriveC.follow(leftDriveA);
        leftDriveB .follow(leftDriveA);
        rightDriveB.follow(rightDriveA);
        rightDriveC.follow(rightDriveA);



        rightDriveA.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightDriveB.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightDriveC.setIdleMode(CANSparkMax.IdleMode.kBrake);

        leftDriveA.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftDriveB.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftDriveB.setIdleMode(CANSparkMax.IdleMode.kBrake);



        rightDriveA.setInverted(true);
    }

    @Override
    protected void printSmartDashboardInternal() {
        super.printSmartDashboardInternal();
    }

    @Override
    protected void setMotorPowerPercentageInternal(double leftPower, double rightPower) {



    }

    @Override
    public double getDistanceRatioLeft() {
        return super.getDistanceRatioLeft();
    }

    @Override
    public double getDistanceTicksLeft() {
        return super.getDistanceTicksLeft();
    }

    @Override
    public double getDistanceRatioRight() {
        return super.getDistanceRatioRight();
    }

    @Override
    public double getDistanceTicksRight() {
        return super.getDistanceTicksRight();
    }

    @Override
    public void resetEncoders() {
        super.resetEncoders();
    }

    @Override
    public void resetEncoderLeft() {
        super.resetEncoderLeft();
    }

    @Override
    public void resetEncoderRight() {
        super.resetEncoderRight();
    }

    @Override
    public double getYaw() {
        return super.getYaw();
    }

    @Override
    public double getPitch() {
        return super.getPitch();
    }

    @Override
    public double getRoll() {
        return super.getRoll();
    }

    @Override
    public void resetAttitude() {
        super.resetAttitude();
    }

    @Override
    protected void setShooterPowerPercentageInternal(double upperPower, double lowerPower) {
        super.setShooterPowerPercentageInternal(upperPower, lowerPower);
    }

    @Override
    protected void spinControlPanelInternal(double power) {
        super.spinControlPanelInternal(power);
    }

    @Override
    public char getCurrentControlPanelColor() {
        return super.getCurrentControlPanelColor();
    }

    @Override
    public Lidar getLidarSubsystem() {
        return super.getLidarSubsystem();
    }

    @Override
    public Double getLidarDistanceInchesFront() {
        return super.getLidarDistanceInchesFront();
    }

    @Override
    public Double getLidarDistanceInchesRear() {
        return super.getLidarDistanceInchesRear();
    }

    @Override
    public Double getLidarDistanceInchesLeft() {
        return super.getLidarDistanceInchesLeft();
    }

    @Override
    public Double getLidarDistanceInchesRight() {
        return super.getLidarDistanceInchesRight();
    }
}
