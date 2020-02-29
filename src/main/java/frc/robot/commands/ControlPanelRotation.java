package frc.robot.commands;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.genericrobot.GenericRobot;
import frc.robot.genericrobot.WheelOfFortune;


public class ControlPanelRotation extends GenericCommand{

    public static CANSparkMax spinner = new CANSparkMax(9, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static CANEncoder spinnerEncoder = new CANEncoder(spinner);
    int[] colorFreq = new int[]{0, 0, 0, 0}; // R, G, B, Y
    WheelOfFortune controlPanel = new WheelOfFortune();
    Color endColor = new WheelOfFortune.GoodColor(0, 0, 0, "Default");
    boolean lookingForRed = true;
    boolean lookingForGreen = true;
    boolean lookingForBlue = true;
    boolean lookingForYellow = true;

    @Override
    public void begin(GenericRobot robot){
        endColor = controlPanel.getInferredColor();
        spinner.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    @Override
    public void step(GenericRobot robot){
        spinner.set(0.6);
        if(controlPanel.getInferredColor() == WheelOfFortune.kRedTarget && lookingForRed){
            lookingForRed = false;
            lookingForBlue = false;
            lookingForGreen = true;
            lookingForYellow = false;
            colorFreq[0]++;
        }
        if(controlPanel.getInferredColor() == WheelOfFortune.kGreenTarget && lookingForGreen){
            lookingForRed = false;
            lookingForBlue = true;
            lookingForGreen = false;
            lookingForYellow = false;
            colorFreq[1]++;
        }
        if(controlPanel.getInferredColor() == WheelOfFortune.kBlueTarget && lookingForBlue){
            lookingForRed = false;
            lookingForBlue = false;
            lookingForGreen = false;
            lookingForYellow = true;
            colorFreq[2]++;
        }
        if(controlPanel.getInferredColor() == WheelOfFortune.kYellowTarget && lookingForYellow){
            lookingForRed = true;
            lookingForBlue = false;
            lookingForGreen = false;
            lookingForYellow = false;
            colorFreq[3]++;
        }

        if(colorFreq[0] >= 6 && colorFreq[1] >= 6 && colorFreq[2] >= 6 && colorFreq[3] >= 6 && controlPanel.getInferredColor() == endColor){
            spinner.set(0.0);
        }
    }
}
