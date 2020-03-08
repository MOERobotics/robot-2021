package frc.robot.genericrobot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    private double climbVerticalPortPower     = 0;
    private double climbVerticalStarboardPower     = 0;
    private double collectorPower         = 0;
    private double indexerPower           = 0;
    private ShifterState gear             = ShifterState.UNKNOWN;
    private ShooterState shooterState     = UNKNOWN;
    public long escalatorSpacing = 40;

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
        SmartDashboard.putBoolean ("Escalator Sensor Medium High", getEscalatorSensorMediumHigh());
        SmartDashboard.putBoolean ("Escalator Sensor High" , getEscalatorSensorHigh()                   );
        SmartDashboard.putNumber  ("Escalator Power"      , escalatorPower                           );
        SmartDashboard.putNumber  ("Indexer Power"        , indexerPower                             );
        SmartDashboard.putString  ("Shooter State"        , shooterState.toString()                  );
        SmartDashboard.putNumber  ("Upper Shooter Power"  , shooterUpperPower                        );
        SmartDashboard.putNumber  ("Lower Shooter Power"  , shooterLowerPower                        );
        SmartDashboard.putNumber  ("Upper Shooter Target Velocity"  , shooterUpperRPM                        );
        SmartDashboard.putNumber  ("Lower Shooter Target Velocity"  , shooterLowerRPM                        );
        SmartDashboard.putNumber  ("Upper Shooter Velocity",getShooterVelocityRPMUpper()             );
        SmartDashboard.putNumber  ("Lower Shooter Velocity",getShooterVelocityRPMLower()             );
        SmartDashboard.putString  ("Shooter Speed Setting" ,shooterSpeedPresetName.name());
        SmartDashboard.putBoolean ("Ready To Shoot"       , readyToShoot()                           );

        SmartDashboard.putNumber  ("Angle Adjust Power"   , angleAdjusterPower                       );

        SmartDashboard.putNumber  ("Climber Vert Port Power"   , climbVerticalPortPower                       );
        SmartDashboard.putNumber  ("Climber Vert Stb Power"   , climbVerticalStarboardPower                       );
        SmartDashboard.putNumber  ("Climber Horiz Power"  , climbBalancePower);
        SmartDashboard.putNumber("Climber Port Ticks", getClimberPortTicks());
        SmartDashboard.putNumber("Climber Starboard Ticks", getClimberStarboardTicks());
        SmartDashboard.putNumber("Climber Port Current", getClimberVerticalPortCurrent());
        SmartDashboard.putNumber("Climber Starboard Current", getClimberVerticalStarboardCurrent());



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

        SmartDashboard.putNumber("Camera Tilt", getCameraTilt());

        printSmartDashboardInternal();
    }

    protected void printSmartDashboardInternal() { }

    public final void updateMotorPowers(){

        if (Math.abs(escalatorPower) < 1.0e-6){
            setEscalatorLights(false);
        } else {
            setEscalatorLights(true);
        }

        if ((getElevationInternal() > getShooterAngleMax()) && (angleAdjusterPower > 0)){
            angleAdjusterPower = 0;
        }

        if ((getElevationInternal() < getShooterAngleMin()) && (angleAdjusterPower < 0)){
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

        setMotorPowerPercentageInternal(leftPower, rightPower);
        spinControlPanelInternal(spinPower);
        setIndexerPowerInternal(indexerPower);
        setCollectorPowerInternal(collectorPower);
        setAngleAdjusterPowerInternal(angleAdjusterPower);
        setEscalatorPowerInternal(escalatorPower);
        setClimbVerticalPortInternal(climbVerticalPortPower);
        setClimbVerticalStarboardInternal(climbVerticalStarboardPower);
        setBalancePowerInternal(climbBalancePower);

        if (shooterState == POWER) {
            setShooterPowerPercentageInternal(shooterUpperPower, shooterLowerPower);
            if ((Math.abs(shooterUpperPower) < 1.0e-6) && (Math.abs(shooterLowerPower) < 1.0e-6)){
                setShooterLights(false);
            } else {
                setShooterLights(true);
            }
        }
        if (shooterState == VELOCITY) {
            setShooterRPMInternal(shooterUpperRPM, shooterLowerRPM);
            if ((Math.abs(shooterUpperRPM) < 1.0e-6) && (Math.abs(shooterLowerRPM) < 1.0e-6)){
                setShooterLights(false);
            } else {
                setShooterLights(true);
            }
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

    public double getClimberVerticalPortCurrent() {
        System.out.println("I don't have any current data for the port climber :'(");
        return 0;
    }

    public double getClimberVerticalStarboardCurrent() {
        System.out.println("I don't have any current data for the starboard side climber :'(");
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

    public double getPIDmaneuverP() {return 0.0;}
    public double getPIDmaneuverI() {return 0.0;}
    public double getPIDmaneuverD() {return 0.0;}

    //***********************************************************************//

    public enum ShooterState {
        POWER,VELOCITY,UNKNOWN;
    }

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
        this.shooterState = POWER;
    }

    public final void setShooterPowerPercentage(
        double power

    ) {
        setShooterPowerPercentage(power, power);
    }

    public ShooterSpeedPresetName shooterSpeedPresetName //?
                = ShooterSpeedPresetName.UNKOWN;



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
        System.out.println("I don't have a shooter :'(");
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
        System.out.println("I don't have a shooter :'(");
        return 0.0;
    }

    public double getShooterVelocityRPMLower(){
        System.out.println("I don't have a shooter :'(");
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



    //***********************************************************************//


    public final void spinControlPanel(
        double power
    ) {
        this.spinPower = power;

    }



    protected void spinControlPanelInternal(
        double power
    ) {
        //System.out.println("I can't spin the control panel :'("); //this is clogging our network
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

    }

    protected void setCollectorPowerInternal(
            double collectorPower
    ){
        System.out.println("I don't have a collector ; (");
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
        System.out.println("I don't have a portside climber :'(");
    }

    protected void setClimbVerticalStarboardInternal(double power) {
        System.out.println("I don't have a starboard side climber :'(");
    }

    public final void setClimbVerticalPower(double power){
        climbVerticalPortPower = power;
        climbVerticalStarboardPower = power;
        setClimbVerticalPortPower(power);
        setClimbVerticalStarboardPower(power);
    }

    protected void climbVerticalInternal (
            double power
    ){
        System.out.println("I don't have a climber ; (");
    }

    public final double getClimberPortTicks(){
        return getClimberPortTicksInternal();

    }
    protected double getClimberPortTicksInternal(){
        System.out.println("I don't have a climber port ;(");
        return 0; //?
    }

    public final double getClimberStarboardTicks(){
        return getClimberStarboardTicksInternal();

    }
    protected double getClimberStarboardTicksInternal(){
        System.out.println("I don't have a climber starboard ;(");
        return 0; //?
    }

    public void resetClimberTicks(){
        resetClimberTicksInternal();
    }

    public void resetClimberTicksInternal() {
        System.out.println("No Climber Encoders to reset");
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
        System.out.println("I don't have a escalator ; (");
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
    protected boolean getEscalatorSensorMediumHighInternal(){
        System.out.println("I don't have a medium high elevator sensor :'(");
        return false;
    }

    public boolean getEscalatorSensorLow(){ return getEscalatorSensorLowInternal();}
    public boolean getEscalatorSensorMedium(){ return getEscalatorSensorMediumInternal();}
    public boolean getEscalatorSensorHigh(){ return getEscalatorSensorHighInternal();}
    public boolean getEscalatorSensorMediumHigh(){ return getEscalatorSensorMediumHighInternal();}


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
        //System.out.println("I don't have a generator shifter ; ("); //this is clogging our network
    }

    //***********************************************************************//

    public final void aimUp(double aimPower){
        setAngleAdjusterPower(aimPower);
    }

    public final void aimDown(double aimPower){
        setAngleAdjusterPower(-aimPower);
    }

    protected double getElevationInternal(){
        System.out.println("I don't have an elevation.");
        return 0.0;
    }

    public double getElevation(){
        return getElevationInternal();
    }

     public final void setAngleAdjusterPower(double power){
        angleAdjusterPower = power;

    }
    protected void setAngleAdjusterPowerInternal(double aimPower){
        System.out.println("I don't have an angle adjuster ;(");
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

    public double getPIDpivotP() {return 0.0;}
    public double getPIDpivotI() {return 0.0;}
    public double getPIDpivotD() {return 0.0;}

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

    public void setClimberBrake(boolean yesNo){
        System.out.println("I don't have a climber");
    }
    public void setShooterLights(boolean onOff){
        System.out.println("I don't have shooter lights");
    }

    public void setEscalatorLights(boolean onOff){
        System.out.println("I don't have escalator lights");
    }

    public void setCameraTilt(double angle){

    }

    public double getCameraTilt(){
        return 0.0;
    }


}
