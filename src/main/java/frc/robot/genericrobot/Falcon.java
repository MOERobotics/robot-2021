package frc.robot.genericrobot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.SPI;

public class Falcon extends GenericRobot{

    AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 50);

    CANSparkMax leftDriveA      = new CANSparkMax(13, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax leftDriveB      = new CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax leftDriveC      = new CANSparkMax(15, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax rightDriveA     = new CANSparkMax(20, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax rightDriveB     = new CANSparkMax( 1, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax rightDriveC     = new CANSparkMax( 2, CANSparkMaxLowLevel.MotorType.kBrushless);

    CANSparkMax climberA        = null;//= new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax climberB        = null;//new CANSparkMax( 3, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax generatorShift  = null;//new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless);

    CANSparkMax shooterA        = new CANSparkMax( 5, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax shooterB        = new CANSparkMax( 4, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax indexer         = new CANSparkMax( 6, CANSparkMaxLowLevel.MotorType.kBrushed);
    CANSparkMax escalator       = new CANSparkMax( 7, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax angleAdj        = new CANSparkMax( 8, CANSparkMaxLowLevel.MotorType.kBrushless);

    CANSparkMax controlPanel    = null;//= new CANSparkMax( 9, CANSparkMaxLowLevel.MotorType.kBrushless);

    CANSparkMax collector       = new CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushed); //Needs t

    CANEncoder encoderRight     = new CANEncoder(rightDriveA);
    CANEncoder encoderLeft      = new CANEncoder( leftDriveA);

    public Falcon() {
        leftDriveC .follow( leftDriveA);
        leftDriveB .follow( leftDriveA);
        rightDriveB.follow(rightDriveA);
        rightDriveC.follow(rightDriveA);

        rightDriveA.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightDriveB.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightDriveC.setIdleMode(CANSparkMax.IdleMode.kBrake);

        leftDriveA .setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftDriveB .setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftDriveB .setIdleMode(CANSparkMax.IdleMode.kBrake);

        rightDriveA.setInverted(true);
    }

    @Override
    protected void printSmartDashboardInternal() {
        super.printSmartDashboardInternal();
    }

    @Override
    protected void setMotorPowerPercentageInternal(double leftPower, double rightPower) {
        rightDriveA.set (rightPower /2);
        rightDriveB.set (rightPower /2);
        rightDriveC.set (rightPower /2);
        leftDriveA.set  (leftPower /2 );
        leftDriveB.set  (leftPower  /2);
        leftDriveB.set  (leftPower  /2);
    }

    @Override
    public double getDistanceRatioLeft() {
        return 0.306;
    }

    @Override
    public double getDistanceTicksLeft() {
        return encoderLeft.getPosition();
    }

    @Override
    public double getDistanceRatioRight() {
        return 0.306;
        }


    @Override
    public double getDistanceTicksRight() {
        return encoderRight.getPosition();
    }

    @Override
    public void resetEncoders() {
        super.resetEncoders();
    }

    @Override
    public void resetEncoderLeft() {
        encoderLeft.setPosition(0.0);
    }

    @Override
    public void resetEncoderRight() {
        encoderRight.setPosition(0.0);
    }

    @Override
    public double getYaw() {
        return navx.getYaw();
    }

    @Override
    public double getPitch() {
        return navx.getPitch();
    }

    @Override
    public double getRoll() {
        return navx.getRoll();
    }

    @Override
    public void resetAttitude() {
        navx.reset();
    }

    @Override
    protected void setShooterPowerPercentageInternal(double upperPower, double lowerPower) {
        shooterA.set(-upperPower);
        shooterB.set(lowerPower);
    }

   /* @Override
    protected void spinControlPanelInternal(double power) {
        controlPanel.set(power);
    }

    @Override
    public char getCurrentControlPanelColor() {
        return super.getCurrentControlPanelColor();
    }

    */

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


    @Override
    protected void setIndexerPowerInternal(double indexerPower) {
        indexer.set(indexerPower);
    }

    @Override
    protected void setCollectorPowerInternal(double collectorPower) {
        collector.set(collectorPower);
    }

    /*@Override
    protected void climbVerticalInternal(double climberPower) {
        climberA.set( climberPower);
        climberB.set(-climberPower);
    }
     */

    @Override
    protected void setEscalatorPowerInternal(double escalatorPower) {
        escalator.set(-escalatorPower);
    }

   /* @Override
    protected void generatorShiftInternal(double shiftPower) {
        generatorShift.set(shiftPower);
    }

    */

    @Override
    protected void setAngleAdjusterPowerInternal(double aimPower) {
        angleAdj.set(aimPower);
    }


}
