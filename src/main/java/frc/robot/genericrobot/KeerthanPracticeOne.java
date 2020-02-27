package frc.robot.genericrobot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SPI;

public class KeerthanPracticeOne extends GenericRobot {

    AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 50);

    CANSparkMax driveRightA = new CANSparkMax(20, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax driveRightB = new CANSparkMax( 1, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax driveLeftA  = new CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax driveLeftB  = new CANSparkMax(15, CANSparkMaxLowLevel.MotorType.kBrushless);

    CANEncoder encoderRight = new CANEncoder(driveRightA);
    CANEncoder encoderLeft  = new CANEncoder(driveLeftA);

    Solenoid shifter = new Solenoid(0);
    Lidar lidar = new Lidar();

    //Limelight
    public KeerthanPracticeOne() {
        driveRightB.follow(driveRightA);
        driveLeftB .follow(driveLeftA);

        driveRightA.setIdleMode(CANSparkMax.IdleMode.kBrake);
        driveRightB.setIdleMode(CANSparkMax.IdleMode.kBrake);
        driveLeftA .setIdleMode(CANSparkMax.IdleMode.kBrake);
        driveLeftB .setIdleMode(CANSparkMax.IdleMode.kBrake);

        driveRightA.setInverted(true);


    }
    private static final ShooterSpeedPreset
            SHOOTER_SPEED_OFF = new ShooterSpeedPreset(0,0),
            SHOOTER_SPEED_SHORT = new ShooterSpeedPreset(2210, 2210),
            SHOOTER_SPEED_MID = new ShooterSpeedPreset(2430, 2430),
            SHOOTER_SPEED_LONG = new ShooterSpeedPreset(4000, 3000), //not final
            SHOOTER_SPEED_YEET = new ShooterSpeedPreset(5000, 5000);




    @Override
    public ShooterSpeedPreset getShooterSpeedPreset(
            ShooterSpeedPresetName speedType
    ){
        switch (speedType){
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
    public double getDistanceTicksLeft() {
        return encoderLeft.getPosition();
    }

    @Override
    public double getDistanceRatioLeft() {
        switch (getShifterState()) {
            case HIGH: return 0.380635;
            case LOW : return 0.77782;
            default  : return 1;
        }
    }

    @Override
    public double getDistanceRatioRight() {
        switch (getShifterState()) {
            case HIGH: return 0.380635;
            case LOW : return 0.77782;
            default  : return 1;
        }
    }

    @Override
    public double getDistanceTicksRight() {
        return encoderRight.getPosition();
    }

    @Override
    public double getPIDmaneuverP(){return 4.0e-2;}

    @Override
    public double getPIDmaneuverI(){return 0.0e-2;}

    @Override
    public double getPIDmaneuverD(){return 1.0e-4;}

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
    public void setMotorPowerPercentageInternal(double leftPower, double rightPower) {
        driveRightA.set (rightPower );
        driveRightB.set (rightPower );
        driveLeftA.set  (leftPower  );
        driveLeftB.set  (leftPower  );
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
    public void resetAttitude() {
        navx.reset();
    }

    @Override
    public void shiftHighInternal() {
        shifter.set(true);
    }

    @Override
    public void shiftLowInternal() {
        shifter.set(false);
    }

    @Override
    public Lidar getLidarSubsystem() {
        return lidar;
    }

    @Override
    public Double getLidarDistanceInchesFront() {
        return lidar.getDistanceInches(2);
    }

    @Override
    public Double getLidarDistanceInchesRear() {
        return lidar.getDistanceInches(0);
    }

    @Override
    public Double getLidarDistanceInchesLeft() {
        return lidar.getDistanceInches(3);
    }

    @Override
    public Double getLidarDistanceInchesRight() {
        return lidar.getDistanceInches(1);
    }

}
