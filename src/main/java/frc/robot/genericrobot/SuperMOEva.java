package frc.robot.genericrobot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;

public class SuperMOEva extends GenericRobot {

    //Drive motors
    TalonSRX driveLA = new TalonSRX(12) {{
        setNeutralMode(NeutralMode.Brake);
    }};
    TalonSRX driveLB = new TalonSRX(13) {{
        setNeutralMode(NeutralMode.Brake);
    }};
    TalonSRX driveRA = new TalonSRX(14) {{
        setNeutralMode(NeutralMode.Brake);
    }};
    TalonSRX driveRB = new TalonSRX(15) {{
        setNeutralMode(NeutralMode.Brake);
    }};
    
    {
        driveLA.setInverted(true);
        driveLB.setInverted(true);
    }

    //Control panel spinners
    TalonSRX rollL = new TalonSRX(11) {{
        setNeutralMode(NeutralMode.Brake);
    }};
    TalonSRX rollR = new TalonSRX(10) {{
        setNeutralMode(NeutralMode.Brake);
    }};

    AHRS navX = new AHRS(SPI.Port.kMXP, (byte) 50);
    Encoder encoderL = new Encoder(0, 1, true, CounterBase.EncodingType.k4X);
    Encoder encoderR = new Encoder(4, 5, true, CounterBase.EncodingType.k4X);


    @Override
    protected void setMotorPowerPercentageInternal(double leftPower, double rightPower) {
        driveLA.set(ControlMode.PercentOutput, leftPower);
        driveLB.set(ControlMode.PercentOutput, leftPower);

        driveRA.set(ControlMode.PercentOutput, rightPower);
        driveRB.set(ControlMode.PercentOutput, rightPower);
    }

    @Override
    protected void spinControlPanelInternal(double power) {
        rollL.set(ControlMode.PercentOutput, power);
        rollR.set(ControlMode.PercentOutput, -power);
    }

    @Override
    public double getDistanceInchesLeft() {
        return encoderL.getDistance();
    }

    @Override
    public double getDistanceInchesRight() {
        return encoderR.getDistance();
    }

    @Override
    public double getYaw() {
        return navX.getYaw();
    }

    @Override
    public double getPitch() {
        return navX.getPitch();
    }

    @Override
    public double getRole() {
        return navX.getRoll();
    }
}
