package frc.robot.genericrobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.*;
import com.revrobotics.*;


import java.util.*;
import java.util.regex.Matcher;

public class WheelOfFortune {

    I2C.Port i2cPort = I2C.Port.kOnboard;
    ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
    static ColorMatch colorMatcher = new ColorMatch();
    static Color kBlueTarget = ColorMatch.makeColor(0.214, 0.521, 0.239);
    static Color kGreenTarget = ColorMatch.makeColor(0.250, 0.543, 0.178);
    static Color kRedTarget = ColorMatch.makeColor(0.437, 0.444, 0.125);
    static Color kYellowTarget = ColorMatch.makeColor(0.351, 0.500, 0.166);





    Queue<Color> colorQueue = new ArrayDeque<Color>(10);
    Color currentInferredColor = null;

    public static void addColors(){
        colorMatcher.addColorMatch(kBlueTarget);
        colorMatcher.addColorMatch(kGreenTarget);
        colorMatcher.addColorMatch(kRedTarget);
        colorMatcher.addColorMatch(kYellowTarget);



    }

    public Color getInstantColor() {
        Color detectedColor = colorSensor.getColor();
        return detectedColor;
    }

    public Color getInferredColor() {

        ColorMatchResult match = colorMatcher.matchClosestColor(getInstantColor());
        return currentInferredColor;

    }

    public void storeColor(Color value) {
        colorQueue.add(value);
        if (colorQueue.size() > 10) {
            colorQueue.remove();
        }
        if (colorQueue.size() <= 9) {
            colorQueue.add(currentInferredColor);
        }
    }
    
    public Color getAndStoreInstantColor() {
        Color colorSensed = getInstantColor();
        storeColor(colorSensed);
        return colorSensed;
    }

    public void queue (Color value) {

        Iterator iterator = colorQueue.iterator();

        int R = 0;
        int G = 0;
        int B = 0;
        int Y = 0;

        while (iterator.hasNext()){
            if (currentInferredColor == kRedTarget) ++R;
            if (currentInferredColor == kBlueTarget) ++B;
            if (currentInferredColor == kGreenTarget) ++G;
            if (currentInferredColor == kYellowTarget) ++Y;
        }

    }






}
