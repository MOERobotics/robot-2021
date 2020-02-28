package frc.robot.genericrobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.*;
import com.revrobotics.*;


import java.util.*;

public class WheelOfFortune {

    public static class GoodColor extends Color {
        String coolName;

        public GoodColor(double red, double green, double blue, String coolName) {
            super(red, green, blue);
            this.coolName = coolName;
        }

        @Override public String toString() {
            return "[Color "+coolName+"]";
        }
    }


    public static ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    public static final Color kBlueTarget   = new GoodColor(0.214, 0.521, 0.239, "Blue"  );
    public static final Color kGreenTarget  = new GoodColor(0.250, 0.543, 0.178, "Green" );
    public static final Color kRedTarget    = new GoodColor(0.437, 0.444, 0.125, "Red"   );
    public static final Color kYellowTarget = new GoodColor(0.351, 0.500, 0.166, "Yellow");
    public static final ColorMatch colorMatcher = new ColorMatch() {{
        addColorMatch(kBlueTarget);
        addColorMatch(kGreenTarget);
        addColorMatch(kRedTarget);
        addColorMatch(kYellowTarget);
    }};

    public Queue<Color> colorQueue = new ArrayDeque<Color>(10);
    public Color currentInferredColor = null;

    public Color getInstantColor() {
       return colorSensor.getColor();
    }

    public Color getInferredColor() {
        return currentInferredColor;
    }

    public void storeColor(Color value) {
        currentInferredColor = Color.kHotPink;
        colorQueue.add(value);
        if (colorQueue.size() > 10) {
            colorQueue.remove();
        }
        int
            R = 0,
            G = 0,
            B = 0,
            Y = 0;

        for (Color color : colorQueue) {
            if (color == kRedTarget    ) ++R;
            if (color == kBlueTarget   ) ++B;
            if (color == kGreenTarget  ) ++G;
            if (color == kYellowTarget ) ++Y;

        }

        if (R >= 6){
            currentInferredColor = kRedTarget;
        }
        if (G >= 6){
            currentInferredColor = kGreenTarget;
        }
        if (B >= 6){
            currentInferredColor = kBlueTarget;
        }
        if (Y >= 6){
            currentInferredColor = kYellowTarget;
        }

    }

    public Color getAndStoreInstantColor() {
        Color colorSensed = getInstantColor();
        storeColor(colorSensed);
        return colorSensed;
    }


}


