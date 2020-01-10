package frc.robot.genericrobot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SPI;

public class KeerthanPracticeOne extends GenericRobot {

    AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 50);
    private Solenoid s1 = new Solenoid(0);
    CANSparkMax driveRightA = new CANSparkMax(20, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax driveRightB = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax driveLeftA = new CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax driveLeftB = new CANSparkMax(15, CANSparkMaxLowLevel.MotorType.kBrushless);

    CANEncoder encoderRight = new CANEncoder(driveRightA);
    CANEncoder encoderLeft = new CANEncoder(driveLeftA);

    @Override
    public double getDistanceRatioLeft()  {
        return 100;
    }

    @Override
    public double getDistanceTicksLeft()  {
        return encoderLeft.getPosition();
    }

    @Override
    public double getDistanceRatioRight()  {
        return 100;
    }

    @Override
    public double getDistanceTicksRight() {
        return encoderRight.getPosition();
    }

    @Override
    public double getDistanceInchesRight() {
        return encoderRight.getPosition();
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
    public void setMotorPowerPercentageInternal(double leftPower, double rightPower) {
        driveRightA.set(rightPower);
        driveRightB.set(rightPower);
        driveLeftA.set(leftPower);
        driveLeftB.set(leftPower);
    }
}
