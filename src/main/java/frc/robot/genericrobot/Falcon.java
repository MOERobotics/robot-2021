package frc.robot.genericrobot;

import com.ctre.phoenix.CANifier;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Util;

public class Falcon extends GenericRobot{

    AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 50);

    CANSparkMax leftDriveA      = new CANSparkMax(13, MotorType.kBrushless);
    CANSparkMax leftDriveB      = new CANSparkMax(14, MotorType.kBrushless);
    CANSparkMax leftDriveC      = new CANSparkMax(15, MotorType.kBrushless);
    CANSparkMax rightDriveA     = new CANSparkMax(20, MotorType.kBrushless);
    CANSparkMax rightDriveB     = new CANSparkMax( 1, MotorType.kBrushless);
    CANSparkMax rightDriveC     = new CANSparkMax( 2, MotorType.kBrushless);

    CANSparkMax climberPort = new CANSparkMax(12, MotorType.kBrushless);
    CANSparkMax climberStarboard = new CANSparkMax( 3, MotorType.kBrushless);
    CANSparkMax generatorShift  = null;//new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless);

    CANSparkMax shooterA                   = new CANSparkMax( 5, MotorType.kBrushless);
    CANPIDController shooterAPIDController = new CANPIDController(shooterA);
    CANSparkMax shooterB                   = new CANSparkMax( 4, MotorType.kBrushless);
    CANPIDController shooterBPIDController = new CANPIDController(shooterB);
    CANSparkMax indexer         = new CANSparkMax( 6, MotorType.kBrushless);
    CANSparkMax escalator       = new CANSparkMax( 7, MotorType.kBrushless);
    CANSparkMax angleAdj        = new CANSparkMax( 8, MotorType.kBrushless);

    CANSparkMax controlPanel    = new CANSparkMax( 9, MotorType.kBrushless);

    CANSparkMax collector       = new CANSparkMax(10, MotorType.kBrushless);

    CANEncoder encoderRight     = new CANEncoder(rightDriveA);
    CANEncoder encoderLeft      = new CANEncoder( leftDriveA);
    CANEncoder encoderShootA    = new CANEncoder(shooterA);
    CANEncoder encoderShootB    = new CANEncoder(shooterB);

    CANEncoder encoderClimbPort = new CANEncoder(climberPort);
    CANEncoder encoderClimbStarboard = new CANEncoder(climberStarboard);

    Lidar lidar = new Lidar();

    PowerDistributionPanel powerPanel = new PowerDistributionPanel();

    Solenoid starboardShooter = new Solenoid(7);
    Solenoid portShooter = new Solenoid(0);
    Solenoid starboardEscalator = new Solenoid(6);
    Solenoid portEscalator = new Solenoid(1);


    private CANDigitalInput angleAdjusterDigitalInputForward;
    private CANDigitalInput angleAdjusterDigitalInputReverse;
    private AnalogInput input = new AnalogInput(0);
    private AnalogPotentiometer elevation = new AnalogPotentiometer(input, 180, 90);

    DigitalInput escalatorSensorLow = new DigitalInput(1);
    DigitalInput escalatorSensorMedium = new DigitalInput(2);
    DigitalInput escalatorSensorHigh = new DigitalInput(3);

    //Servo cameraTilt = new Servo(0);



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

        shooterA.setIdleMode(IdleMode.kCoast);
        shooterB.setIdleMode(IdleMode.kCoast);

        shooterAPIDController.setP (7.50e-5);
        shooterAPIDController.setI (1.00e-6);
        shooterAPIDController.setD (2.00e-2);
        shooterAPIDController.setFF(1.67e-4); //feed forward
        shooterAPIDController.setIZone(500);
        shooterAPIDController.setDFilter(0);

        shooterBPIDController.setP (7.50e-5);
        shooterBPIDController.setI (1.00e-6);
        shooterBPIDController.setD (2.00e-2);
        shooterBPIDController.setFF(1.67e-4);
        shooterBPIDController.setIZone(500);
        shooterBPIDController.setDFilter(0);

        angleAdj.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
        angleAdj.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);

        angleAdjusterDigitalInputForward = angleAdj.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed);
        angleAdjusterDigitalInputReverse = angleAdj.getForwardLimitSwitch(CANDigitalInput.LimitSwitchPolarity.kNormallyClosed);

        climberPort.setIdleMode     (IdleMode.kBrake);
        climberStarboard.setIdleMode(IdleMode.kBrake);

        indexer.setInverted(true);

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
    public double getPIDmaneuverP(){return 1.0e-1;}

    @Override
    public double getPIDmaneuverI(){return 1.0e-2;}

    @Override
    public double getPIDmaneuverD(){return 2.0e-4;}

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


    private static final ShooterSpeedPreset
            SHOOTER_SPEED_OFF   = new ShooterSpeedPreset(   0,    0),
            SHOOTER_SPEED_SHORT = new ShooterSpeedPreset(2285, 2285),
            SHOOTER_SPEED_MID   = new ShooterSpeedPreset(2620, 2620),
            SHOOTER_SPEED_LONG  = new ShooterSpeedPreset(4000, 3000), //not final
            SHOOTER_SPEED_YEET  = new ShooterSpeedPreset(5000, 5000);




    @Override
    public ShooterSpeedPreset getShooterSpeedPreset(
            ShooterSpeedPresetName speedType
    ){
        switch (speedType){
            case SHORT_RANGE : return SHOOTER_SPEED_SHORT;
            case MID_RANGE   : return SHOOTER_SPEED_MID;
            case LONG_RANGE  : return SHOOTER_SPEED_LONG;
            case YEET        : return SHOOTER_SPEED_YEET;
            default          : return SHOOTER_SPEED_OFF;

        }
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

    @Override
    protected void setClimbVerticalStarboardInternal(double power) {
        climberStarboard.set(-power);
    }

    @Override
    protected void setClimbVerticalPortInternal(double power) {
        climberPort.set(power);
    }

    @Override
    public double getClimberVerticalPortPositionMin(){return 10.0;}

    @Override
    public double getClimberVerticalStarboardPositionMin(){return 10.0;}

    @Override
    public double getClimberVerticalPortPositionMax(){return 130.0;}

    @Override
    public double getClimberVerticalStarboardPositionMax(){return 130.0;}

    @Override
    //public double getClimberVerticalStarboardCurrent() {return powerPanel.getCurrent(3);}
    public double getClimberVerticalStarboardAmperage() {return climberStarboard.getOutputCurrent();}

    @Override
    //public double getClimberVerticalPortCurrent() {return powerPanel.getCurrent(12);}
    public double getClimberVerticalPortAmperage() {return climberPort.getOutputCurrent();}

    @Override
    public double getClimberPortTicks() {return Math.abs(encoderClimbPort.getPosition());}

    @Override
    public double getClimberStarboardTicks() {return Math.abs(encoderClimbStarboard.getPosition());}

    @Override
    public void resetClimberTicks() {
        encoderClimbPort.setPosition(0.0);
        encoderClimbStarboard.setPosition(0.0);
    }

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
    protected void setAimAdjusterPowerInternal(double aimPower) {

        angleAdj.set(-aimPower);
}

    @Override
    protected double getAimElevationInternal(){return elevation.get();}

    @Override
    public double getShooterAngleMax(){return 153.0;} //orig 155

    @Override
    public double getShooterAngleMin(){return 114.0;} //orig 113

    @Override
    public double getPIDpivotP() {
        return 4.0e-2;
    }

    @Override
    public double getPIDpivotI() {
        return 1.0e-2;
    }

    @Override
    public double getPIDpivotD() {
        return 1.0e-4;
    }

    @Override
    public boolean getEscalatorSensorLowInternal(){
        return escalatorSensorLow.get();
    }

    @Override
    public boolean getEscalatorSensorMediumInternal(){
        return escalatorSensorMedium.get();
    }

    @Override
    public boolean getEscalatorSensorHighInternal(){
        return escalatorSensorHigh.get();
    }

    @Override
    protected void setClimberBrakeInternal(Util.BrakeModeState state){
        switch (state) {
            case BRAKE:
                climberPort     .setIdleMode(IdleMode.kBrake);
                climberStarboard.setIdleMode(IdleMode.kBrake);
                break;
            case COAST:
                climberPort     .setIdleMode(IdleMode.kCoast);
                climberStarboard.setIdleMode(IdleMode.kCoast);
                break;
        }
    }

    @Override
    public void setShooterLights(boolean onOff){
        portShooter.set(onOff);
        starboardShooter.set(onOff);
    }

    @Override
    public void setEscalatorLights(boolean onOff){
        portEscalator.set(onOff);
        starboardEscalator.set(onOff);
    }

    /*@Override
    public void setCameraTilt(double angle){
        cameraTilt.setAngle(angle);
    }

    @Override
    public double getCameraTilt(){
        return cameraTilt.getAngle();
    }

     */


}

