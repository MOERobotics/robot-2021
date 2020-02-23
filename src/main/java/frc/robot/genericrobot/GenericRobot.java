package frc.robot.genericrobot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Logger;

import static frc.robot.Util.coalesce;
import static frc.robot.genericrobot.GenericRobot.ShooterState.*;

public abstract class GenericRobot {

    private double leftPower              = 0;
    private double rightPower             = 0;
    private double spinPower              = 0;
    private double shooterUpperPower      = 0;
    private double shooterLowerPower      = 0;
    private double shooterUpperRPM        = 0;
    private double shooterLowerRPM        = 0;
    private double angleAdjusterPower     = 0;
    private double climbBalancePower = 0;
    private double escalatorPower = 0;
    private double climbVerticalPower     = 0;
    private double collectorPower         = 0;
    private double indexerPower           = 0;
    private ShifterState gear             = ShifterState.UNKNOWN;
    private ShooterState shooterState     = UNKNOWN;

    public final void printSmartDashboard() {
        SmartDashboard.putNumber  ("Left  Encoder Ticks"  , getDistanceTicksLeft()                   );
        SmartDashboard.putNumber  ("Right Encoder Ticks"  , getDistanceTicksRight()                  );
        SmartDashboard.putNumber  ("Navx Yaw"             , getYaw()                                 );
        SmartDashboard.putNumber  ("Navx Pitch"           , getPitch()                               );
        SmartDashboard.putNumber  ("Navx Roll"            , getRoll()                                );
        SmartDashboard.putNumber  ("Left  Motor Power"    , leftPower                                );
        SmartDashboard.putNumber  ("Right Motor Power"    , rightPower                               );
        SmartDashboard.putNumber  ("Left Encoder Inches"  , getDistanceInchesLeft()                  );
        SmartDashboard.putNumber  ("Right Encoder Inches" , getDistanceInchesRight()                 );
        SmartDashboard.putString  ("Shifter state"        , gear.toString()                          );

        SmartDashboard.putNumber  ("Collector Power"      , collectorPower                           );
        SmartDashboard.putNumber  ("Escalator Power"      , escalatorPower);
        SmartDashboard.putBoolean ("Escalator Sensor Low"  , getEscalatorSensorLow()                   );
        SmartDashboard.putBoolean ("Escalator Sensor Medium", getEscalatorSensorMedium()                   );
        SmartDashboard.putBoolean ("Escalator Sensor High" , getEscalatorSensorHigh()                   );
        SmartDashboard.putNumber  ("Escalator Power"      , escalatorPower                           );
        SmartDashboard.putNumber  ("Indexer Power"        , indexerPower                             );
        SmartDashboard.putString  ("Shooter State"        , shooterState.toString()                  );
        SmartDashboard.putNumber  ("Upper Shooter Power"  , shooterUpperPower                        );
        SmartDashboard.putNumber  ("Lower Shooter Power"  , shooterLowerPower                        );
        SmartDashboard.putNumber  ("Upper Shooter Velocity",getShooterVelocityRPMUpper()             );
        SmartDashboard.putNumber  ("Lower Shooter Velocity",getShooterVelocityRPMLower()             );
        SmartDashboard.putBoolean ("Ready To Shoot"       , readyToShoot()                           );

        SmartDashboard.putNumber  ("Angle Adjust Power"   , angleAdjusterPower                       );

        SmartDashboard.putNumber  ("Climber Vert Power"   , climbVerticalPower                       );
        SmartDashboard.putNumber  ("Climber Horiz Power"  , climbBalancePower                        );

        SmartDashboard.putNumber  ("Control Panel Power"  , spinPower                                );

        SmartDashboard.putBoolean ("Lidar Locked"         , isLidarBusLocked()                       );
        SmartDashboard.putNumber  ("Lidar Front"          , coalesce(getLidarDistanceInchesFront(), -9999.0) );
        SmartDashboard.putNumber  ("Lidar Rear"           , coalesce(getLidarDistanceInchesRear (), -9999.0) );
        SmartDashboard.putNumber  ("Lidar Left"           , coalesce(getLidarDistanceInchesLeft (), -9999.0) );
        SmartDashboard.putNumber  ("Lidar Right"          , coalesce(getLidarDistanceInchesRight(), -9999.0) );

        SmartDashboard.putNumber  ("Limelight X"          , limelight.getLimelightX   ()             );
        SmartDashboard.putNumber  ("Limelight Y"          , limelight.getLimelightY   ()             );
        SmartDashboard.putNumber  ("Limelight A"          , limelight.getLimelightArea()             );

        SmartDashboard.putNumber  ("Elevation"            , getElevation());
        SmartDashboard.putBoolean("Shooter Over Limit", (getElevation() > getShooterAngleMax()));
        SmartDashboard.putBoolean("Shooter Under Limit", (getElevation() < getShooterAngleMin()));

        printSmartDashboardInternal();


        Logger.logValue("LEFTPOWER", "Power is currently" + leftPower, leftPower);
        Logger.logValue("RIGHTPOWER", "Power is currently" + rightPower, rightPower);
        Logger.logValue("SPINPOWER", "Power is currently" + spinPower, spinPower);
        Logger.logValue("SHOOTERUPPER", "Shooter power is currently" + shooterUpperPower, shooterUpperPower );
        Logger.logValue("SHOOTERLOWER", "Shooter power is currently" + shooterLowerPower, shooterLowerPower);
        Logger.logValue("ANGLEADJ", "Angle adjuster power is currently" + angleAdjusterPower, angleAdjusterPower);
        Logger.logValue("CLIMBBALANCE", "Climb balance power is currently" + climbBalancePower, climbBalancePower);
        Logger.logValue("CLIMBVERTICAL", "Climb vertical power is currently" + climbVerticalPower, climbVerticalPower);
        Logger.logValue("ESCALATOR", "Escalator power is currently" + escalatorPower, escalatorPower);
        Logger.logValue("COLLECTOR", "Collector power is currently" + collectorPower, collectorPower);
        Logger.logValue("INDEXER", "Indexer power is currently" + indexerPower, indexerPower);

        //Logger.logTTL("LEFTENCODER", "Left encoder ticks are" + getDistanceTicksLeft(), 1000);
        //Logger.logTTL("RIGHTENCODER", "Right encoder ticks are" + getDistanceTicksRight(), 1000);
        //Logger.logTTL("NAVXYAW", "Navx's yaw reading is" + getYaw(), 1000);
        //Logger.logTTL("NAVXPITCH", "Navx's pitch reading is" + getPitch(), 1000);
        //Logger.logTTL("NAVXROLL", "Navx's roll reading is" + getRoll(), 1000);
        //Logger.logTTL("LEFTENCODEIN", "Left encoder inches are" + getDistanceInchesLeft(), 1000);
        //Logger.logTTL("RIGHTENCODEIN", "Right encoder inches are" + getDistanceInchesRight(), 1000);
        //Logger.logTTL("SHIFTERSTATE", "The shifter state is " + getShifterState(), 1000);
        //Logger.logTTL("UPPERVELOCITY", "The upper shooter velocity is" + getShooterVelocityRPMUpper(), 1000);
        //Logger.logTTL("LOWERVELOCITY", "The lower shooter velocity is" + getShooterVelocityRPMLower(), 1000);
        //Logger.logTTL("LIDARLOCKED", "Is the lidar locked?" + isLidarBusLocked(), 1000);
        //Logger.logTTL("LIDARFRONTDIS", "The lidar front distance (inches) is" + getLidarDistanceInchesFront(), 1000);
        //Logger.logTTL("LIDARREARTDIS", "The lidar rear distance (inches) is" + getLidarDistanceInchesRear(), 1000);
        //Logger.logTTL("LIDARLEFTDIS", "The lidar left distance (inches) is" + getLidarDistanceInchesLeft(), 1000);
        //Logger.logTTL("LIDARRIGHTDIS", "The lidar right distance (inches) is" + getLidarDistanceInchesRight(), 1000);
        //Logger.logTTL("LIMELIGHTX", "The limelight X axis is" + limelight.getLimelightX(), 1000);
        //Logger.logTTL("LIMELIGHTY", "The limelight Y axis is" + limelight.getLimelightY(), 1000);
        //Logger.logTTL("LIMELIGHTA", "The limelight area is" + limelight.getLimelightArea(), 1000);
        //Logger.logTTL("ELEVATION", "The elevation is" + getElevation(), 1000);
        //Logger.logTTL("SHOOTEROVER", "The shooter over limit is" + (getElevation() > getShooterAngleMax()), 1000);
        //Logger.logTTL("SHOOTERUNDER", "The shooter under limit is" + (getElevation() < getShooterAngleMin()), 1000);
































    }

    protected void printSmartDashboardInternal() { }

    public final void updateMotorPowers(){

        if ((getElevationInternal() > getShooterAngleMax()) && (angleAdjusterPower > 0)){
            angleAdjusterPower = 0;
        }

        if ((getElevationInternal() < getShooterAngleMin()) && (angleAdjusterPower < 0)){
            angleAdjusterPower = 0;
        }




        setMotorPowerPercentageInternal(leftPower, rightPower);
        spinControlPanelInternal(spinPower);
        setIndexerPowerInternal(indexerPower);
        setCollectorPowerInternal(collectorPower);
        setAngleAdjusterPowerInternal(angleAdjusterPower);
        setEscalatorPowerInternal(escalatorPower);
        climbVerticalInternal(climbVerticalPower);
        setBalancePowerInternal(climbBalancePower);

        if (shooterState == POWER)
            setShooterPowerPercentageInternal(shooterUpperPower, shooterLowerPower);
        if (shooterState == VELOCITY)
            setShooterRPMInternal(shooterUpperRPM, shooterLowerRPM);
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

    //***********************************************************************//

    public enum ShifterState {
        HIGH,LOW,UNKNOWN;
    }

    public final void shiftHigh(){
        gear = ShifterState.HIGH;
        shiftHighInternal();

    }

    public final void shiftLow(){
        gear = ShifterState.LOW;
        shiftLowInternal();
    }

    protected void shiftHighInternal() {
        Logger.log("NOSHIFTERHIGH", "I don't have a shifter");
    }

    protected void shiftLowInternal() {
        Logger.log("NOSHIFTERLOW", "I don't have a shifter");

    }


    public ShifterState getShifterState() {
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

    public enum ShooterState {
        POWER,VELOCITY,UNKNOWN;
    }

    public final void setShooterPowerPercentage(
        double upperPower,
        double lowerPower
    ) {
        this.shooterUpperPower = upperPower;
        this.shooterLowerPower = lowerPower;
        this.shooterState = POWER;
    }

    public final void setShooterPowerPercentage(
        double power
    ) {
        setShooterPowerPercentage(power, power);
    }


    public void setShooterRPM(double upperRPM, double lowerRPM) {
        this.shooterUpperRPM = upperRPM;
        this.shooterLowerRPM = lowerRPM;
        this.shooterState = VELOCITY;
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
        System.out.println("I don't have a shooter :'(");
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

    protected boolean readyToShootInternal(){
        System.out.println("I don't have a shooter :'(");
        return true;
    }

    public boolean readyToShoot(){
        return readyToShootInternal();
    }
    //in falcon, check if both motors match target RPM within a fraction of a percentage (10th of a percent)
    //abs((RPM - targetRPM)/targetRPM) should be between 0.999 and 1.001

    public ShooterState getShooterState() {
        return shooterState;
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

    public final void climbUp(double power){
        climbVertical(power) ;
    }

    public final void climbDown(double power){
        climbVertical(-power);
    }

    public final void stopClimb() {
        climbVertical(0.0);
    }

    public final void climbVertical(double power){
        climbVerticalPower = power;

    }

    protected void climbVerticalInternal (
            double power
    ){
        Logger.log("CLIMBVERTINT","I don't have a climber ; (");
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
        System.out.println("I don't have a low elevator sensor :'(");
        return false;
    }

    protected boolean getEscalatorSensorMediumInternal(){
        System.out.println("I don't have a medium elevator sensor :'(");
        return false;
    }
    protected boolean getEscalatorSensorHighInternal(){
        System.out.println("I don't have a high elevator sensor :'(");
        return false;
    }

    public boolean getEscalatorSensorLow(){ return getEscalatorSensorLowInternal();}
    public boolean getEscalatorSensorMedium(){ return getEscalatorSensorMediumInternal();}
    public boolean getEscalatorSensorHigh(){ return getEscalatorSensorHighInternal();}


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
        setAngleAdjusterPower(aimPower);
    }

    public final void aimDown(double aimPower){
        setAngleAdjusterPower(-aimPower);
    }

    protected double getElevationInternal(){
        Logger.log("ELEVATIONINT","I don't have an elevation.");
        return 0.0;
    }

    public double getElevation(){
        return getElevationInternal();
    }

     public final void setAngleAdjusterPower(double power){
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

    public double getLimelightMinpower(){
        System.out.println("I don't have a limelight :'(");
        return 0;
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
