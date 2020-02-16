package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.genericrobot.GenericRobot;
import frc.robot.genericrobot.WheelOfFortune;


public class ControlPanelRotation extends GenericCommand{

    CANSparkMax spinner = new CANSparkMax(99, CANSparkMaxLowLevel.MotorType.kBrushless);
    int[] colorFreq = new int[]{0, 0, 0, 0}; // R, G, B, Y
    WheelOfFortune controlPanel = new WheelOfFortune();
    Color endColor = new WheelOfFortune.GoodColor(0, 0, 0, "Default");

    @Override
    public void begin(GenericRobot robot){
        endColor = controlPanel.getInferredColor();
    }

    @Override
    public void step(GenericRobot robot){
        spinner.set(0.1);
        if(controlPanel.getInferredColor() == WheelOfFortune.kRedTarget){
            colorFreq[0]++;
        }
        if(controlPanel.getInferredColor() == WheelOfFortune.kGreenTarget){
            colorFreq[1]++;
        }
        if(controlPanel.getInferredColor() == WheelOfFortune.kBlueTarget){
            colorFreq[2]++;
        }
        if(controlPanel.getInferredColor() == WheelOfFortune.kYellowTarget){
            colorFreq[3]++;
        }

        if(colorFreq[0] >= 6 && colorFreq[1] >= 6 && colorFreq[2] >= 6 && colorFreq[3] >= 6 && controlPanel.getInferredColor() == endColor){
            spinner.set(0);
        }
    }
}
