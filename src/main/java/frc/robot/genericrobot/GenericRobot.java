package frc.robot.genericrobot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Logger;
import frc.robot.Util.*;

import static frc.robot.Util.ALMOST_ZERO;
import static frc.robot.Util.coalesce;
import static frc.robot.Util.MotorControlMethod.*;

public abstract class GenericRobot {

    private double             leftPower                   = 0;
    private double             rightPower                  = 0;
    private double             spinPower                   = 0;
    private double             shooterUpperPower           = 0;
    private double             shooterLowerPower           = 0;
    private double             shooterUpperRPM             = 0;
    private double             shooterLowerRPM             = 0;
    private double             angleAdjusterPower          = 0;
    private double             climbBalancePower           = 0;
    private double             escalatorPower              = 0;
    private double             climbVerticalPortPower      = 0;
    private double             climbVerticalStarboardPower = 0;
    private double             collectorPower              = 0;
    private double             indexerPower                = 0;
    private GearShiftState     gear                        = GearShiftState.UNKNOWN;
    private MotorControlMethod shooterControlMethod = UNKNOWN;
    private BrakeModeState     climberBrakeModeState       = BrakeModeState.UNKNOWN;
    public  long               escalatorSpacing            = 40;

    public final void printSmartDashboard() {
        SmartDashboard.putString  ("Robot name"                    , getClass().getSimpleName()                        );


        SmartDashboard.putNumber  ("Left  Encoder Ticks"           , getDistanceTicksLeft()                            );
        SmartDashboard.putNumber  ("Right Encoder Ticks"           , getDistanceTicksRight()                           );
        SmartDashboard.putNumber  ("Navx Yaw"                      , getYaw()                                          );
        SmartDashboard.putNumber  ("Navx Pitch"                    , getPitch()                                        );
        SmartDashboard.putNumber  ("Navx Roll"                     , getRoll()                                         );
        SmartDashboard.putNumber  ("Left  Motor Power"             , leftPower                                         );
        SmartDashboard.putNumber  ("Right Motor Power"             , rightPower                                        );
        SmartDashboard.putNumber  ("Left Encoder Inches"           , getDistanceInchesLeft()                           );
        SmartDashboard.putNumber  ("Right Encoder Inches"          , getDistanceInchesRight()                          );
        SmartDashboard.putString  ("Shifter state"                 , gear.toString()                                   );

        SmartDashboard.putNumber  ("Collector Power"               , collectorPower                                    );
        SmartDashboard.putNumber  ("Escalator Power"               , escalatorPower                                    );
        SmartDashboard.putBoolean ("Escalator Sensor Low"          , getEscalatorSensorLow()                           );
        SmartDashboard.putBoolean ("Escalator Sensor Medium"       , getEscalatorSensorMedium()                        );
        SmartDashboard.putBoolean ("Escalator Sensor High"         , getEscalatorSensorHigh()                          );
        SmartDashboard.putNumber  ("Escalator Power"               , escalatorPower                                    );
        SmartDashboard.putNumber  ("Indexer Power"                 , indexerPower                                      );
        SmartDashboard.putString  ("Shooter Control Method"        , shooterControlMethod.toString()                   );
        SmartDashboard.putNumber  ("Upper Shooter Power"           , shooterUpperPower                                 );
        SmartDashboard.putNumber  ("Lower Shooter Power"           , shooterLowerPower                                 );
        SmartDashboard.putNumber  ("Upper Shooter Target Velocity" , shooterUpperRPM                                   );
        SmartDashboard.putNumber  ("Lower Shooter Target Velocity" , shooterLowerRPM                                   );
        SmartDashboard.putNumber  ("Upper Shooter Velocity"        , getShooterVelocityRPMUpper()                      );
        SmartDashboard.putNumber  ("Lower Shooter Velocity"        , getShooterVelocityRPMLower()                      );
        SmartDashboard.putString  ("Shooter Speed Setting"         , shooterSpeedPresetName.name()                     );
        SmartDashboard.putBoolean ("Ready To Shoot"                , readyToShoot()                                    );

        SmartDashboard.putNumber  ("Angle Adjust Power"            , angleAdjusterPower                                );

        SmartDashboard.putNumber  ("Climber Vert Port Power"       , climbVerticalPortPower                            );
        SmartDashboard.putNumber  ("Climber Vert Stb Power"        , climbVerticalStarboardPower                       );
        SmartDashboard.putNumber  ("Climber Horiz Power"           , climbBalancePower                                 );
        SmartDashboard.putNumber  ("Climber Port Ticks"            , getClimberPortTicks()                             );
        SmartDashboard.putNumber  ("Climber Starboard Ticks"       , getClimberStarboardTicks()                        );
        SmartDashboard.putNumber  ("Climber Port Current"          , getClimberVerticalPortAmperage()                  );
        SmartDashboard.putNumber  ("Climber Starboard Current"     , getClimberVerticalStarboardAmperage()             );
        SmartDashboard.putString  ("Climber brake mode"            , climberBrakeModeState.toString()                  );

        SmartDashboard.putNumber  ("Control Panel Power"           , spinPower                                         );

        SmartDashboard.putBoolean ("Lidar Locked"                  , isLidarBusLocked()                                );
        SmartDashboard.putNumber  ("Lidar Front"                   , coalesce(getLidarDistanceInchesFront() , -9999.0) );
        SmartDashboard.putNumber  ("Lidar Rear"                    , coalesce(getLidarDistanceInchesRear () , -9999.0) );
        SmartDashboard.putNumber  ("Lidar Left"                    , coalesce(getLidarDistanceInchesLeft () , -9999.0) );
        SmartDashboard.putNumber  ("Lidar Right"                   , coalesce(getLidarDistanceInchesRight() , -9999.0) );

        SmartDashboard.putNumber  ("Limelight X"                   , limelight.getLimelightX   ()                      );
        SmartDashboard.putNumber  ("Limelight Y"                   , limelight.getLimelightY   ()                      );
        SmartDashboard.putNumber  ("Limelight A"                   , limelight.getLimelightArea()                      );

        SmartDashboard.putNumber  ("Elevation"                     , getAimElevation()                                    );
        SmartDashboard.putBoolean ("Shooter Over Limit"            , (getAimElevation() > getShooterAngleMax())           );
        SmartDashboard.putBoolean ("Shooter Under Limit"           , (getAimElevation() < getShooterAngleMin())           );

        SmartDashboard.putNumber  ("Camera Tilt"                   , getCameraTilt()                                   );

        printSmartDashboardInternal();


        Logger.logValue("LEFTPOWER", "Power is currently" + leftPower, leftPower);
        Logger.logValue("RIGHTPOWER", "Power is currently" + rightPower, rightPower);
        Logger.logValue("SPINPOWER", "Power is currently" + spinPower, spinPower);
        Logger.logValue("SHOOTERUPPER", "Shooter power is currently" + shooterUpperPower, shooterUpperPower );
        Logger.logValue("SHOOTERLOWER", "Shooter power is currently" + shooterLowerPower, shooterLowerPower);
        Logger.logValue("ANGLEADJ", "Angle adjuster power is currently" + angleAdjusterPower, angleAdjusterPower);
        Logger.logValue("CLIMBHORIZBALANCE", "Climb horizontal  power is currently" + climbBalancePower, climbBalancePower);
        Logger.logValue("CLIMBVERTPORTPWR", "Climb vertical port power is currently" + climbVerticalPortPower, climbVerticalPortPower);
        Logger.logValue("CLIMBVERTSTBPWR", "Climb vertical starboard power is currently" + climbVerticalStarboardPower, climbVerticalStarboardPower);
        Logger.logValue("CLIMBPORTTICS", "Climber port ticks are:" + getClimberPortTicks(), getClimberPortTicks());
        Logger.logValue("CLIMBSTARTICS", "Climber starboard ticks are:" + getClimberStarboardTicks(), getClimberStarboardTicks());
        Logger.logValue("CLIMBPORTCURR", "The climber port current is:" + getClimberVerticalPortAmperage(), getClimberVerticalPortAmperage());
        Logger.logValue("CLIMBSTARCURR", "The climber starboard current is:" + getClimberVerticalStarboardAmperage(), getClimberVerticalStarboardAmperage());



        Logger.logValue("ESCALATOR", "Escalator power is currently" + escalatorPower, escalatorPower);
        Logger.logValue("COLLECTOR", "Collector power is currently" + collectorPower, collectorPower);
        Logger.logValue("INDEXER", "Indexer power is currently" + indexerPower, indexerPower);
        Logger.logValue("CAMERATILT", "The Camera Tilt is currently" + getCameraTilt(), getCameraTilt());
        Logger.logValue("READYTOSHOOT", "Are we ready to shoot:" + readyToShoot(), readyToShoot());


        Logger.logTTL("LEFTENCODER", "Left encoder ticks are" + getDistanceTicksLeft(), 1000);
        Logger.logTTL("RIGHTENCODER", "Right encoder ticks are" + getDistanceTicksRight(), 1000);
        Logger.logTTL("NAVXYAW", "Navx's yaw reading is" + getYaw(), 1000);
        Logger.logTTL("NAVXPITCH", "Navx's pitch reading is" + getPitch(), 1000);
        Logger.logTTL("NAVXROLL", "Navx's roll reading is" + getRoll(), 1000);
        Logger.logTTL("LEFTENCODEIN", "Left encoder inches are" + getDistanceInchesLeft(), 1000);
        Logger.logTTL("RIGHTENCODEIN", "Right encoder inches are" + getDistanceInchesRight(), 1000);
        Logger.logTTL("SHIFTERSTATE", "The shifter state is " + getShifterState(), 1000);
        Logger.logTTL("UPPERVELOCITY", "The upper shooter velocity is" + getShooterVelocityRPMUpper(), 1000);
        Logger.logTTL("LOWERVELOCITY", "The lower shooter velocity is" + getShooterVelocityRPMLower(), 1000);
        Logger.logTTL("LIDARLOCKED", "Is the lidar locked?" + isLidarBusLocked(), 1000);
        Logger.logTTL("LIDARFRONTDIS", "The lidar front distance (inches) is" + getLidarDistanceInchesFront(), 1000);
        Logger.logTTL("LIDARREARTDIS", "The lidar rear distance (inches) is" + getLidarDistanceInchesRear(), 1000);
        Logger.logTTL("LIDARLEFTDIS", "The lidar left distance (inches) is" + getLidarDistanceInchesLeft(), 1000);
        Logger.logTTL("LIDARRIGHTDIS", "The lidar right distance (inches) is" + getLidarDistanceInchesRight(), 1000);
        Logger.logTTL("LIMELIGHTX", "The limelight X axis is" + limelight.getLimelightX(), 1000);
        Logger.logTTL("LIMELIGHTY", "The limelight Y axis is" + limelight.getLimelightY(), 1000);
        Logger.logTTL("LIMELIGHTA", "The limelight area is" + limelight.getLimelightArea(), 1000);
        Logger.logTTL("ELEVATION", "The elevation is" + getAimElevation(), 1000);
        Logger.logTTL("SHOOTEROVER", "The shooter over limit is" + (getAimElevation() > getShooterAngleMax()), 1000);
        Logger.logTTL("SHOOTERUNDER", "The shooter under limit is" + (getAimElevation() < getShooterAngleMin()), 1000);
    }

    protected void printSmartDashboardInternal() { }

    public final void updateMotorPowers(){


        if ((getAimElevationInternal() > getShooterAngleMax()) && (angleAdjusterPower > 0)){
            angleAdjusterPower = 0;
        }

        if ((getAimElevationInternal() < getShooterAngleMin()) && (angleAdjusterPower < 0)){
            angleAdjusterPower = 0;
        }

        if ((getClimberPortTicks() < getClimberVerticalPortPositionMin()) && (climbVerticalPortPower < 0)){
            climbVerticalPortPower = 0;
        }

        if ((getClimberStarboardTicks() < getClimberVerticalStarboardPositionMin()) && (climbVerticalStarboardPower < 0)){
            climbVerticalStarboardPower = 0;
        }

        if ((getClimberPortTicks() > getClimberVerticalPortPositionMax()) && (climbVerticalPortPower > 0)){
            climbVerticalPortPower = 0;
        }

        if ((getClimberStarboardTicks() > getClimberVerticalStarboardPositionMax()) && (climbVerticalStarboardPower > 0)){
            climbVerticalStarboardPower = 0;
        }

        setEscalatorLights(Math.abs(escalatorPower) > ALMOST_ZERO);
        setMotorPowerPercentageInternal(leftPower, rightPower);
        spinControlPanelInternal(spinPower);
        setIndexerPowerInternal(indexerPower);
        setCollectorPowerInternal(collectorPower);
        setAngleAdjusterPowerInternal(angleAdjusterPower);
        setEscalatorPowerInternal(escalatorPower);
        setClimbVerticalPortInternal(climbVerticalPortPower);
        setClimbVerticalStarboardInternal(climbVerticalStarboardPower);
        setBalancePowerInternal(climbBalancePower);
        setCameraTiltDegreesInternal(cameraTiltAngle);
        setClimberBrakeInternal(climberBrakeModeState);

        switch (shooterControlMethod) {
            case POWER:
                setShooterPowerPercentageInternal(shooterUpperPower, shooterLowerPower);
                setShooterLights((Math.abs(shooterUpperPower) > ALMOST_ZERO) || (Math.abs(shooterLowerPower) > ALMOST_ZERO));
                break;
            case VELOCITY:
                setShooterRPMInternal(shooterUpperRPM, shooterLowerRPM);
                setShooterLights((Math.abs(shooterUpperRPM) > ALMOST_ZERO) || (Math.abs(shooterLowerRPM) > ALMOST_ZERO));
                break;
        }
    }

    //***********************************************************************//

    public final void setMotorPowerPercentage(
        double leftPower,
        double rightPower
    ) {
        this.leftPower = leftPower;
        this.rightPower = rightPower;
    }

    protected abstract void setMotorPowerPercentageInternal(
        double leftPower,
        double rightPower
    );

    public final void driveForward(
        double power
    ) {
        setMotorPowerPercentage(power, power);
    }

    public final void driveReverse(
        double power
    ) {
        setMotorPowerPercentage(-power, -power);
    }

    public final void driveLeftInPlace(
        double power
    ) {
        setMotorPowerPercentage(-power, power);
    }

    public final void driveRightInPlace(
        double power
    ) {
        setMotorPowerPercentage(power, -power);
    }

    public final double getMotorPowerLeft() {
        return leftPower;
    }

    public final double getMotorPowerRight() {
        return rightPower;
    }

    public double getPIDpivotP() {return 0.0;}
    public double getPIDpivotI() {return 0.0;}
    public double getPIDpivotD() {return 0.0;}

    //***********************************************************************//

    public final void shiftHigh(){
        gear = GearShiftState.HIGH_GEAR;
        shiftHighInternal();

    }

    public final void shiftLow(){
        gear = GearShiftState.LOW_GEAR;
        shiftLowInternal();
    }

    protected void shiftHighInternal() {
        Logger.log("NOSHIFTERHIGH", "I don't have a shifter");
    }

    protected void shiftLowInternal() {
        Logger.log("NOSHIFTERLOW", "I don't have a shifter");

    }

    public GearShiftState getShifterState() {
        return gear;
    }


    //***********************************************************************//

    public final double getDistanceInchesLeft() {
        return getDistanceTicksLeft() / getDistanceRatioLeft();
    }

    public final double getDistanceInchesRight() {
        return getDistanceTicksRight() / getDistanceRatioRight();
    }

    public double getDistanceRatioLeft() {
        Logger.log("NORATIOLEFT", "I don't have a encoder");
        return 0;
    }

    public double getDistanceTicksLeft() {
        Logger.log("NOTICKSLEFT", "I don't have a encoder");
        return 0;
    }

    public double getDistanceRatioRight() {

        Logger.log("NORATIORIGHT", "I don't have a encoder");
        return 0;
    }

    public double getDistanceTicksRight() {
        Logger.log("NOTICKSRIGHT", "I don't have a encoder");
        return 0;
    }

    public double getClimberVerticalPortAmperage() {
        Logger.log("CLIMBVERTAMP","I don't have any current data for the port climber :'(");
        return 0;
    }

    public double getClimberVerticalStarboardAmperage() {
        Logger.log("CLIMBVERSTARAMP","I don't have any current data for the starboard side climber :'(");
        return 0;
    }

    public double getClimberVerticalPortPositionMin(){return 1.0;}
    public double getClimberVerticalStarboardPositionMin(){return 1.0;}

    public double getClimberVerticalPortPositionMax(){return 1.0e6;}
    public double getClimberVerticalStarboardPositionMax(){return 1.0e6;}


    public void resetEncoders() {
        resetEncoderLeft();
        resetEncoderRight();
    }

    public void resetEncoderLeft() {
        Logger.log("ENCODERLEFT", "I don't have an encoder");
    }

    public void resetEncoderRight() {

        Logger.log("ENCODERRIGHT","I don't have encoders");
    }

    public double getYaw() {
        Logger.log("GETYAW","I don't have a navx :'(");
        return 0;
    }

    public double getPitch() {
        Logger.log("GETPITCH","I don't have a navx :'(");
        return 0;
    }

    public double getRoll() {
        Logger.log("GETROLL","I don't have a navx :'(");
        return 0;
    }

    public void resetAttitude() {
        Logger.log("ATTITUDERESET","I don't have a navx :'(");
    }

    public double getPIDmaneuverP() {return 0.0;}
    public double getPIDmaneuverI() {return 0.0;}
    public double getPIDmaneuverD() {return 0.0;}

    //***********************************************************************//

    public enum ShooterSpeedPresetName {
        UNKOWN, SHORT_RANGE, MID_RANGE, LONG_RANGE, YEET;
    }

    public static class ShooterSpeedPreset{
        public final int
            upperRPM,
            lowerRPM;
        public ShooterSpeedPreset(
            int upperRPM,
            int lowerRPM
        ){
            this.upperRPM = upperRPM;
            this.lowerRPM = lowerRPM;

        }
    }

    public final void setShooterPowerPercentage(
        double upperPower,
        double lowerPower
    ) {
        this.shooterUpperPower = upperPower;
        this.shooterLowerPower = lowerPower;
        this.shooterControlMethod = POWER;
    }

    public final void setShooterPowerPercentage(
        double power

    ) {
        setShooterPowerPercentage(power, power);
    }

    public ShooterSpeedPresetName shooterSpeedPresetName
        = ShooterSpeedPresetName.UNKOWN;



    public void setShooterRPM(double upperRPM, double lowerRPM) {
        this.shooterUpperRPM = upperRPM;
        this.shooterLowerRPM = lowerRPM;
        this.shooterControlMethod = VELOCITY;
    }

    public void setShooterRPM(double RPM) {setShooterRPM(RPM,RPM); }

    protected void setShooterPowerPercentageInternal(
        double upperPower,
        double lowerPower
    ) {
        Logger.log("SETSHOOTERINT","I don't have a shooter :'(");
    }

    protected void setShooterRPMInternal(
        double upperRPM,
        double lowerRPM
    ) {
        Logger.log("SHOOTERRPMINT","I don't have a shooter :'(");
    }


    public final double getShooterPowerUpper() {
        return shooterUpperPower;
    }

    public final double getShooterPowerLower() {
        return shooterLowerPower;
    }

    public double getShooterVelocityRPMUpper(){
        Logger.log("SHOOTERUP","I don't have a shooter :'(");
        return 0.0;
    }

    public double getShooterVelocityRPMLower(){
        Logger.log("SHOOTERLOW","I don't have a shooter :'(");
        return 0.0;
    }

    public double getShooterTargetRPMLower(){
        return this.shooterLowerRPM;
    }

    public double getShooterTargetRPMUpper(){
        return this.shooterUpperRPM;
    }


    private long startTime= 0;
    public boolean readyToShoot(){
        double targetUpper = getShooterTargetRPMUpper();
        double targetLower = getShooterTargetRPMLower();
        double errorUpper = Math.abs((getShooterVelocityRPMUpper() + targetUpper) / targetUpper); //upperRPM is negative for shooting operation, think about this later
        double errorLower = Math.abs((getShooterVelocityRPMLower() - targetLower) / targetLower);
        if((errorUpper < 0.02) && (errorLower < 0.02)) {
            if (System.currentTimeMillis() - startTime > 100) return true;
        } else {
            startTime = System.currentTimeMillis();
        }
        return false;
    }

    public MotorControlMethod getShooterControlMethod() {
        return shooterControlMethod;
    }

    public final ShooterSpeedPresetName getShooterSpeedConstant() {
        return shooterSpeedPresetName;
    }

    public final void setShooterSpeedPresetName(ShooterSpeedPresetName speedPresetName){
        this.shooterSpeedPresetName = speedPresetName;
    }

    public final void setShooterRPMFromSpeedConst(){
        setShooterRPMFromSpeedConstInternal(shooterSpeedPresetName);
    }

    public final void setShooterRPMFromSpeedConst(ShooterSpeedPresetName speedPresetName){
        this.shooterSpeedPresetName = speedPresetName;
        setShooterRPMFromSpeedConstInternal(speedPresetName);
    }

    public final void setShooterRPMFromSpeedConstInternal(ShooterSpeedPresetName speedPresetName) {
        ShooterSpeedPreset speed = getShooterSpeedPreset(speedPresetName);
        setShooterRPM(
                speed.upperRPM,
                speed.lowerRPM
        );
    }

    private static final ShooterSpeedPreset
        SHOOTER_SPEED_OFF = new ShooterSpeedPreset(0,0);

    public ShooterSpeedPreset getShooterSpeedPreset(
        ShooterSpeedPresetName speedType
    ){
        return SHOOTER_SPEED_OFF;
    }

    public void setShooterLights(boolean onOff){
        Logger.log("SHOOTERLIGHTS","I don't have shooter lights");
    }


    //***********************************************************************//


    public final void spinControlPanel(
        double power
    ) {
        this.spinPower = power;

    }



    protected void spinControlPanelInternal(
        double power
    ) {
        Logger.log("CTRLPANELINT","I can't spin the control panel :'(");
    }

    public final double getControlPanelSpinnerPower() {
        return spinPower;
    }

    public char getCurrentControlPanelColor() {
        Logger.log("CURRCTRLCOLOR","I don't have a color sensor :'(");
        return '?';
    }

    //***********************************************************************//

    public final void indexerLoad(double indexerPower){
        setIndexerPower(indexerPower);
    }

    public final void indexerUnload(double indexerPower){
        setIndexerPower(-indexerPower);
    }

    public final void setIndexerPower(double power){
        indexerPower = power;

    }
    protected void setIndexerPowerInternal(
            double indexerPower

    ){
        Logger.log("INDEXINT","I don't have an Indexer ; (");
    }

    //***********************************************************************//

    public final void collectorIn(double collectorPower) {
        setCollectorPower(collectorPower);
    }

    public final void collectorOut(double collectorPower){
        setCollectorPower(-collectorPower);
    }

    public final void setCollectorPower(double power){
        collectorPower = power;

    }

    protected void setCollectorPowerInternal(
            double collectorPower
    ){
        Logger.log("COLLECTORINT","I don't have a collector ; (");
    }

    //***********************************************************************//

    public final void lowerClimberArms(double power){
        setClimbVerticalPower(-power) ;
    }

    public final void raiseClimberArms(double power){
        setClimbVerticalPower(power);
    }

    public final void stopClimb() {
        setClimbVerticalPower(0.0);
    }

    public final void setClimbVerticalPortPower(double power){
        climbVerticalPortPower = power;
    }

    public final void setClimbVerticalStarboardPower(double power){
        climbVerticalStarboardPower = power;
    }

    protected void setClimbVerticalPortInternal(double power) {
        Logger.log("SETVERTPORTINT","I don't have a portside climber :'(");
    }

    protected void setClimbVerticalStarboardInternal(double power) {
       Logger.log("SETVERTSTARINT","I don't have a starboard side climber :'(");
    }

    public final void setClimbVerticalPower(double power){
        setClimbVerticalPortPower(power);
        setClimbVerticalStarboardPower(power);
    }

    public double getClimberPortTicks(){
        Logger.log("CLIMBPORTTICKS","I don't have a climber starboard ;(");
        return 0; //?
    }

    public double getClimberStarboardTicks(){
        Logger.log("CLIMBSTARTICKS","I don't have a climber starboard ;(");
        return 0; //?
    }

    public void resetClimberTicks(){
        Logger.log("CLIMBTICKS","No Climber Encoders to reset");
    }


    public void setClimberBrake(BrakeModeState state) {
        climberBrakeModeState = state;
    }
    protected void setClimberBrakeInternal(BrakeModeState state) {
        Logger.log("CLIMBRAKEINT","I don't have a climber that can enable brake mode :(");
    }

    //***********************************************************************//

    public final void escalatorUp(double power){
        setEscalatorPower(power) ;
    }

    public final void escalatorDown(double power){
        setEscalatorPower(-power);
    }


    public final void setEscalatorPower(double power){
        escalatorPower = power;

    }

    protected void setEscalatorPowerInternal (
            double power
    ){
        Logger.log("ESCALATORINT","I don't have a escalator ; (");
    }


    protected boolean getEscalatorSensorLowInternal(){
        Logger.log("ESCSENSLOWINT","I don't have a low elevator sensor :'(");
        return false;
    }

    protected boolean getEscalatorSensorMediumInternal(){
        Logger.log("ESCSENSMEDINT","I don't have a medium elevator sensor :'(");
        return false;
    }
    protected boolean getEscalatorSensorHighInternal(){
        Logger.log("ESCSENSHIGHINT","I don't have a high elevator sensor :'(");
        return false;
    }

    public boolean getEscalatorSensorLow(){ return getEscalatorSensorLowInternal();}
    public boolean getEscalatorSensorMedium(){ return getEscalatorSensorMediumInternal();}
    public boolean getEscalatorSensorHigh(){ return getEscalatorSensorHighInternal();}


    public void setEscalatorLights(boolean onOff){
        Logger.log("ESCALIGHTS","I don't have escalator lights");
    }

    //***********************************************************************//

    public final void climberBalanceLeft(double power){
        setBalancePower(-power);
    }

    public final void climberBalanceRight(double power){
        setBalancePower(power);
    }

    public final void setBalancePower(double power){
        climbBalancePower = power;
    }

    protected void setBalancePowerInternal(
            double shiftPower
    ){
        Logger.log("BALANCEINT","I don't have a generator shifter ; (");
    }

    //***********************************************************************//

    public final void aimUp(double aimPower){
        setAimAdjusterPower(aimPower);
    }

    public final void aimDown(double aimPower){
        setAimAdjusterPower(-aimPower);
    }

    protected double getAimElevationInternal(){
        Logger.log("ELEVATIONINT","I don't have an elevation.");
        return 0.0;
    }

    public double getAimElevation(){
        return getAimElevationInternal();
    }

     public final void setAimAdjusterPower(double power){
        angleAdjusterPower = power;

    }
    protected void setAngleAdjusterPowerInternal(double aimPower){
        Logger.log("ANGLEADJINT","I don't have an angle adjuster ;(");
    }

    public double getShooterAngleMax(){return 0;}

    public double getShooterAngleMin(){return 0;}

    //***********************************************************************//


    //Todo: Yeet into own class
    public final Limelight limelight = new Limelight();
    public static class Limelight {

        public NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");

        private Limelight() {
            table.getEntry("pipeline").setNumber(0);
        }

        public double getLimelightX() {
            return tx.getDouble(0.0);
        }

        public double getLimelightY() {
            return ty.getDouble(0.0);
        }

        public double getLimelightArea() {
            return ta.getDouble(0.0);
        }
    }


    public double cameraTiltAngle;
    public final void setCameraTiltDegrees(double angle) {
        cameraTiltAngle = angle;
    }

    protected void setCameraTiltDegreesInternal(double angle) {
        Logger.log("CAMTILTDEGINT","I don't have a camera servo :'(");
    }

    public double getCameraTilt(){
        return cameraTiltAngle;
    }


    //***********************************************************************//

    public Lidar getLidarSubsystem() {
       Logger.log("LIDARSUB","I don't have a lidar bus :'(");
        return null;
    }

    public final boolean isLidarBusLocked() {
        Lidar lidarSystem = getLidarSubsystem();
        if (lidarSystem == null) return false;
        return lidarSystem.isLocked();
    }

    public Double getLidarDistanceInchesFront() {
        Logger.log("NOLIDARFRONT", "I don't lave a lidar");
        return null;
    }

    public Double getLidarDistanceInchesRear() {
        Logger.log("NOLIDARREAR", "I don't lave a lidar");
        return null;
    }
    public Double getLidarDistanceInchesLeft() {
        Logger.log("NOLIDARLEFT", "I don't lave a lidar");
        return null;
    }

    public Double getLidarDistanceInchesRight() {
        Logger.log("NOLIDARRIGHT", "I don't lave a lidar");
        return null;
    }

    //***********************************************************************//



}
