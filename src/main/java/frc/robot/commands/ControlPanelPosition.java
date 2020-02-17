package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.genericrobot.GenericRobot;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.genericrobot.WheelOfFortune;

public class ControlPanelPosition extends GenericCommand{

    CANSparkMax spinner = new CANSparkMax(99, CANSparkMaxLowLevel.MotorType.kBrushless);
    WheelOfFortune controlPanel = new WheelOfFortune();
    Color endColor = new WheelOfFortune.GoodColor(0, 0, 0, "Default");

    @Override
    public void begin(GenericRobot robot){
        String gameData;
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        if(gameData.length() > 0)
        {
            switch (gameData.charAt(0))
            {
                case 'B' :
                    //Blue case code
                    endColor = WheelOfFortune.kBlueTarget;
                    break;
                case 'G' :
                    //Green case code
                    endColor = WheelOfFortune.kGreenTarget;
                    break;
                case 'R' :
                    //Red case code
                    endColor = WheelOfFortune.kRedTarget;
                    break;
                case 'Y' :
                    //Yellow case code
                    endColor = WheelOfFortune.kYellowTarget;
                    break;
                default :
                    //This is corrupt data
                    break;
            }
        } else {
            //Code for no data received yet
        }
    }

    @Override
    public void step(GenericRobot robot){

    }
}
