package frc.robot.genericrobot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Util.coalesce;

public abstract class GenericRobot {

    private double leftPower              = 0;
    private double rightPower             = 0;
    private double spinPower              = 0;
    private double shooterUpperPower      = 0;
    private double shooterLowerPower      = 0;
    private double angleAdjusterPower     = 0;
    private double climbBalancePower = 0;
    private double escalatorPower = 0;
    private double climbVerticalPower     = 0;
    private double collectorPower         = 0;
    private double indexerPower           = 0;
    private ShifterState gear             = ShifterState.UNKNOWN;

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
        SmartDashboard.putString  ("Shifter state"        , getShifterState().toString()             );

        SmartDashboard.putNumber  ("Collector Power"      , collectorPower                           );
        SmartDashboard.putNumber  ("Escalator Power"      , escalatorPower);
        SmartDashboard.putNumber  ("Indexer Power"        , indexerPower                             );
        SmartDashboard.putNumber  ("Upper Shooter Power"  , shooterUpperPower                        );
        SmartDashboard.putNumber  ("Lower Shooter Power"  , shooterLowerPower                        );
        SmartDashboard.putNumber  ("Upper Shooter Velocity", getShooterVelocityUpper()               );
        SmartDashboard.putNumber  ("Lower Shooter Velocity", getShooterVelcoityLower()               );

        SmartDashboard.putNumber  ("Angle Adjust Power"   , angleAdjusterPower                       );

        SmartDashboard.putNumber  ("Climber Vert Power"   , climbVerticalPower                       );
        SmartDashboard.putNumber  ("Climber Horiz Power"  , climbBalancePower);

        SmartDashboard.putNumber  ("Control Panel Power"  , spinPower                                );

var lidar = getLidarSubsystem();
        SmartDashboard.putBoolean ("Lidar Locked"         , isLidarBusLocked()                       );
        SmartDashboard.putNumber  ("Lidar Front"          , coalesce(getLidarDistanceInchesFront(), -9999.0) );
        SmartDashboard.putNumber  ("Lidar Rear"           , coalesce(getLidarDistanceInchesRear (), -9999.0) );
        SmartDashboard.putNumber  ("Lidar Left"           , coalesce(getLidarDistanceInchesLeft (), -9999.0) );
        SmartDashboard.putNumber  ("Lidar Right"          , coalesce(getLidarDistanceInchesRight(), -9999.0) );
        SmartDashboard.putNumber  ("Lidar 0 Raw"          , coalesce(lidar.getDistance(0), -9999) );
        SmartDashboard.putNumber  ("Lidar 1 Raw"           , coalesce(lidar.getDistance (1), -9999) );
        SmartDashboard.putNumber  ("Lidar 2 Raw"           , coalesce(lidar.getDistance (2), -9999) );
        SmartDashboard.putNumber  ("Lidar 3 Raw"          , coalesce(lidar.getDistance(3), -9999) );

        SmartDashboard.putNumber  ("Limelight X"          , limelight.getLimelightX   ()             );
        SmartDashboard.putNumber  ("Limelight Y"          , limelight.getLimelightY   ()             );
        SmartDashboard.putNumber  ("Limelight A"          , limelight.getLimelightArea()             );

        printSmartDashboardInternal();
    }

    protected void printSmartDashboardInternal() { }

    //***********************************************************************//

    public final void setMotorPowerPercentage(
        double leftPower,
        double rightPower
    ) {
        this.leftPower = leftPower;
        this.rightPower = rightPower;
        setMotorPowerPercentageInternal(
                leftPower,
                rightPower
        );
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
        System.out.println("I don't have a shifter ;(");
    }

    protected void shiftLowInternal() {
        System.out.println("I don't have a shifter ;(");
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
        System.out.println("I don't have an encoder :'(");
        return 0;
    }

    public double getDistanceTicksLeft() {
        System.out.println("I don't have an encoder :'(");
        return 0;
    }

    public double getDistanceRatioRight() {

        System.out.println("I don't have an encoder :'(");
        return 0;
    }

    public double getDistanceTicksRight() {
        System.out.println("I don't have an encoder :'(");
        return 0;
    }

    public void resetEncoders() {
        resetEncoderLeft();
        resetEncoderRight();
    }

    public void resetEncoderLeft() {
        System.out.println("I don't have encoders");
    }

    public void resetEncoderRight() {
        System.out.println("I don't have encoders");
    }

    public double getYaw() {
        System.out.println("I don't have a navx :'(");
        return 0;
    }

    public double getPitch() {
        System.out.println("I don't have a navx :'(");
        return 0;
    }

    public double getRoll() {
        System.out.println("I don't have a navx :'(");
        return 0;
    }

    public void resetAttitude() {
        System.out.println("I don't have a navx :'(");
    }

    //***********************************************************************//

    public final void setShooterPowerPercentage(
        double upperPower,
        double lowerPower
    ) {
        this.shooterUpperPower = upperPower;
        this.shooterLowerPower = lowerPower;
        setShooterPowerPercentageInternal(
                upperPower,
                lowerPower
        );
    }

    public final void setShooterPowerPercentage(
        double power
    ) {
        setShooterPowerPercentage(power, power);
    }

    protected void setShooterPowerPercentageInternal(
        double upperPower,
        double lowerPower
    ) {
        System.out.println("I don't have a shooter :'(");
    }

    public final double getShooterPowerUpper() {
        return shooterUpperPower;
    }

    public final double getShooterPowerLower() {
        return shooterLowerPower;
    }

    public double getShooterVelocityUpper(){
        System.out.println("I don't have a shooter :'(");
        return 0.0;
    }

    public double getShooterVelcoityLower(){
        System.out.println("I don't have a shooter :'(");
        return 0.0;
    }


    //***********************************************************************//

    public final void spinControlPanel(
        double power
    ) {
        this.spinPower = power;
        spinControlPanelInternal(power);
    }



    protected void spinControlPanelInternal(
        double power
    ) {
        System.out.println("I can't spin the control panel :'(");
    }

    public final double getControlPanelSpinnerPower() {
        return spinPower;
    }

    public char getCurrentControlPanelColor() {
        System.out.println("I don't have a color sensor :'(");
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
        setIndexerPowerInternal(power);
    }
    protected void setIndexerPowerInternal(
            double indexerPower

    ){
        System.out.println("I don't have an Indexer ; (");
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
        setCollectorPowerInternal(power);
    }

    protected void setCollectorPowerInternal(
            double collectorPower
    ){
        System.out.println("I don't have a collector ; (");
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
        climbVerticalInternal(power);
    }

    protected void climbVerticalInternal (
            double power
    ){
        System.out.println("I don't have a climber ; (");
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
        setEscalatorPowerInternal(power);
    }

    protected void setEscalatorPowerInternal (
            double power
    ){
        System.out.println("I don't have a escalator ; (");
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
        setBalancePowerInternal(power);
    }

    protected void setBalancePowerInternal(
            double shiftPower
    ){
        System.out.println("I don't have a generator shifter ; (");
    }

    //***********************************************************************//

    public final void aimUp(double aimPower){
        setAngleAdjusterPower(aimPower);
    }

    public final void aimDown(double aimPower){
        setAngleAdjusterPower(-aimPower);
    }

     public final void setAngleAdjusterPower(double power){
        angleAdjusterPower = power;
        setAngleAdjusterPowerInternal(power);
    }
    protected void setAngleAdjusterPowerInternal(double aimPower){
        System.out.println("I don't have an angle adjuster ;(");
    }

    //***********************************************************************//


    //Todo: Yeet into own class
    public final Limelight limelight = new Limelight();
    public static class Limelight {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");

        private Limelight() {
            table.getEntry("pipeline").setNumber(1);
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

    //***********************************************************************//

    public Lidar getLidarSubsystem() {
        System.out.println("I don't have a lidar bus :'(");
        return null;
    }

    public final boolean isLidarBusLocked() {
        Lidar lidarSystem = getLidarSubsystem();
        if (lidarSystem == null) return false;
        return lidarSystem.isLocked();
    }

    public Double getLidarDistanceInchesFront() {
        System.out.println("I don't have a front lidar :'(");
        return null;
    }

    public Double getLidarDistanceInchesRear() {
        System.out.println("I don't have a rear lidar :'(");
        return null;
    }
    public Double getLidarDistanceInchesLeft() {
        System.out.println("I don't have a left lidar :'(");
        return null;
    }

    public Double getLidarDistanceInchesRight() {
        System.out.println("I don't have a right lidar :'(");
        return null;
    }

    //***********************************************************************//



}
