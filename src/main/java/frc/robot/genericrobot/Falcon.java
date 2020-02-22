package frc.robot.genericrobot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Falcon extends GenericRobot{

    AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 50);

    CANSparkMax leftDriveA      = new CANSparkMax(13, MotorType.kBrushless);
    CANSparkMax leftDriveB      = new CANSparkMax(14, MotorType.kBrushless);
    CANSparkMax leftDriveC      = new CANSparkMax(15, MotorType.kBrushless);
    CANSparkMax rightDriveA     = new CANSparkMax(20, MotorType.kBrushless);
    CANSparkMax rightDriveB     = new CANSparkMax( 1, MotorType.kBrushless);
    CANSparkMax rightDriveC     = new CANSparkMax( 2, MotorType.kBrushless);

    CANSparkMax climberA        = null;//= new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax climberB        = null;//new CANSparkMax( 3, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax generatorShift  = null;//new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless);

    CANSparkMax shooterA                   = new CANSparkMax( 5, MotorType.kBrushless);
    CANPIDController shooterAPIDController = new CANPIDController(shooterA);
    CANSparkMax shooterB                   = new CANSparkMax( 4, MotorType.kBrushless);
    CANPIDController shooterBPIDController = new CANPIDController(shooterB);
    CANSparkMax indexer         = new CANSparkMax( 6, MotorType.kBrushed);
    CANSparkMax escalator       = new CANSparkMax( 7, MotorType.kBrushless);
    CANSparkMax angleAdj        = new CANSparkMax( 8, MotorType.kBrushless);

    CANSparkMax controlPanel    = null;//= new CANSparkMax( 9, CANSparkMaxLowLevel.MotorType.kBrushless);

    CANSparkMax collector       = new CANSparkMax(10, MotorType.kBrushless);

    CANEncoder encoderRight     = new CANEncoder(rightDriveA);
    CANEncoder encoderLeft      = new CANEncoder( leftDriveA);
    CANEncoder encoderShootA    = new CANEncoder(shooterA);
    CANEncoder encoderShootB    = new CANEncoder(shooterB);
    Lidar lidar = new Lidar();

    private CANDigitalInput angleAdjusterDigitalInputForward;
    private CANDigitalInput angleAdjusterDigitalInputReverse;
    private AnalogInput input = new AnalogInput(0);
    private AnalogPotentiometer elevation = new AnalogPotentiometer(input, 180, 90);

    DigitalInput escalatorSensorLow = new DigitalInput(1);
    DigitalInput escalatorSensorMedium = new DigitalInput(2);
    DigitalInput escalatorSensorHigh = new DigitalInput(3);


    public Falcon() {

        rightDriveA.setIdleMode(IdleMode.kBrake);
        rightDriveB.setIdleMode(IdleMode.kBrake);
        rightDriveC.setIdleMode(IdleMode.kBrake);

        leftDriveA .setIdleMode(IdleMode.kBrake);
        leftDriveB .setIdleMode(IdleMode.kBrake);
        leftDriveC .setIdleMode(IdleMode.kBrake);

        rightDriveA.setInverted(true);
        rightDriveB.setInverted(true);
        rightDriveC.setInverted(true);

        collector.setInverted(true);

        escalator.setIdleMode(IdleMode.kBrake);

        // REMOVE BEFORE FLIGHT... Just for testing.
        shooterA.setIdleMode(IdleMode.kBrake);
        shooterB.setIdleMode(IdleMode.kBrake);

        shooterAPIDController.setP(7.5e-5);
        shooterAPIDController.setI(1.0e-6);
        shooterAPIDController.setD(1.0e-2);
        shooterAPIDController.setFF(1.67e-4);
        shooterAPIDController.setIZone(500);
        shooterAPIDController.setDFilter(0);

        shooterBPIDController.setP(7.5e-5);
        shooterBPIDController.setI(1.0e-6);
        shooterBPIDController.setD(1.0e-2);
        shooterBPIDController.setFF(1.67e-4);
        shooterBPIDController.setIZone(500);
        shooterBPIDController.setDFilter(0);

        angleAdj.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
        angleAdj.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);

        angleAdjusterDigitalInputForward = angleAdj.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed);
        angleAdjusterDigitalInputReverse = angleAdj.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed);

    }

    @Override
    protected void printSmartDashboardInternal() {
        super.printSmartDashboardInternal();
    }

    @Override
    protected void setMotorPowerPercentageInternal(double leftPower, double rightPower) {
        rightDriveA.set (rightPower);
        rightDriveB.set (rightPower);
        rightDriveC.set (rightPower);
         leftDriveA.set (leftPower);
         leftDriveB.set (leftPower);
         leftDriveC.set (leftPower);
    }
    @Override
    public double getShooterVelocityRPMUpper(){
        return encoderShootA.getVelocity();
    }
    @Override
    public double getShooterVelocityRPMLower(){
        return encoderShootB.getVelocity();
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
    public void setShooterRPMInternal(double upperRPM, double lowerRPM) {
        shooterAPIDController.setReference(-upperRPM, ControlType.kVelocity);
        shooterBPIDController.setReference( lowerRPM, ControlType.kVelocity);
    }

    @Override
    protected void setShooterPowerPercentageInternal(double upperPower, double lowerPower) {
        shooterA.set(-upperPower);
        shooterB.set(lowerPower);
    }

    @Override
    protected boolean readyToShootInternal(){
        double targetUpper = getShooterTargetRPMUpper();
        double targetLower = getShooterTargetRPMLower();
        boolean readyToShoot = true;
        double errorUpper = Math.abs((getShooterVelocityRPMUpper() + targetUpper) / targetUpper); //upperRPM is negative for shooting operation, think about this later
        double errorLower = Math.abs((getShooterVelocityRPMLower() - targetLower) / targetLower);
        SmartDashboard.putNumber("errorUpper", (errorUpper * 100));
        SmartDashboard.putNumber("errorLower", (errorLower * 100));
        if((errorUpper > 5.0e-2) || (errorLower > 5.0e-2)){
            readyToShoot = false;
        }
        return readyToShoot;
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
    public Lidar getLidarSubsystem() {return lidar; }

    @Override
    public Double getLidarDistanceInchesFront() {
        return lidar.getDistanceInches(3);
    }

    @Override
    public Double getLidarDistanceInchesRear() {
        return lidar.getDistanceInches(2);
    }

    @Override
    public Double getLidarDistanceInchesLeft() {
        return lidar.getDistanceInches(0);
    }

    @Override
    public Double getLidarDistanceInchesRight() {
        return lidar.getDistanceInches(1);
    }

    @Override
    protected void setIndexerPowerInternal(double indexerPower) {
        indexer.set(indexerPower);
    }

    @Override
    protected void setCollectorPowerInternal(double collectorPower) {

        collector.set(-collectorPower);
    }

    /*@Override
    protected void climbVerticalInternal(double climberPower) {
        climberA.set( climberPower);
        climberB.set(-climberPower);
    }
     */

    @Override
    protected void setEscalatorPowerInternal(double escalatorPower) {
        escalator.set(escalatorPower);
    }

   /* @Override
    protected void generatorShiftInternal(double shiftPower) {
        generatorShift.set(shiftPower);
    }

    */

    @Override
    protected void setAngleAdjusterPowerInternal(double aimPower) {

        angleAdj.set(-aimPower);
}

    @Override
    protected double getElevationInternal(){return elevation.get();}

    @Override
    public double getShooterAngleMax(){return 153.0;} //orig 155

    @Override
    public double getShooterAngleMin(){return 114.0;} //orig 113

    @Override
    public double getLimelightMinpower() {
        return .03;
    }
    @Override
    public boolean getElevatorSensorLowInternal(){
        return escalatorSensorLow.get();
    }

    @Override
    public boolean getElevatorSensorMediumInternal(){
        return escalatorSensorMedium.get();
    }

    @Override
    public boolean getElevatorSensorHighInternal(){
        return escalatorSensorHigh.get();
    }
=========
    public double getShooterAngleMax(){return 153.0;} //orig 155

    @Override
    public double getShooterAngleMin(){return 114.0;} //orig 113
>>>>>>>>> Temporary merge branch 2


}


