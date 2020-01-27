package frc.robot.genericrobot;
import edu.wpi.first.wpilibj.util.*;

import java.util.*;

public class WheelOfFortune {

    Queue<Color> colorQueue = new ArrayDeque<Color>(10);
    Color currentInferredColor = null;

    public Color getInstantColor() {
        return null;
    }

    public Color getInferredColor() {
        return currentInferredColor;
    }

    public void storeColor(Color value) {
        colorQueue.add(value);
        if (colorQueue.size() > 9) {
            colorQueue.remove();
        }
        //TODO: Code to find the most common color
    }

    public Color getAndStoreInstantColor() {
        Color colorSensed = getInstantColor();
        storeColor(colorSensed);
        return colorSensed;
    }

}
