package frc.robot.genericrobot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SPI;

public class KeerthanPracticeOne extends GenericRobot {

    AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 50);

    CANSparkMax driveRightA = new CANSparkMax(20, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax driveRightB = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax driveLeftA = new CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax driveLeftB = new CANSparkMax(15, CANSparkMaxLowLevel.MotorType.kBrushless);

    //Right motor ratio = .78615
    //Left motor ratio = .78949
    CANEncoder encoderRight = new CANEncoder(driveRightA);
    CANEncoder encoderLeft = new CANEncoder(driveLeftA);

    Solenoid shifter = new Solenoid(0);

    public KeerthanPracticeOne() {
        driveRightB.follow(driveRightA);
        driveLeftB.follow(driveLeftA);

        driveRightA.setIdleMode(CANSparkMax.IdleMode.kBrake);
        driveRightB.setIdleMode(CANSparkMax.IdleMode.kBrake);
        driveLeftA.setIdleMode(CANSparkMax.IdleMode.kBrake);
        driveLeftB.setIdleMode(CANSparkMax.IdleMode.kBrake);

        driveRightA.setInverted(true);
    }

    @Override
    public double getDistanceTicksLeft() {
        return encoderLeft.getPosition();
    }

    @Override
    public double getDistanceRatioLeft() {
        if (getShifterState() == false) {
            //High gear ratio
            return 0.78949;
        } else {
            //Low gear ratio
            return 0.38913;
        }
    }

    @Override
    public double getDistanceRatioRight() {
        if (getShifterState() == false) {
            return 0.78615;
        } else {
            return 0.37414;
        }
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
    public boolean getShifterState() {
        return shifter.get();
    }

    @Override
    public void shiftHigh() {
        shifter.set(true);
    }

    @Override
    public void shiftLow() {
        shifter.set(false);
    }
}
