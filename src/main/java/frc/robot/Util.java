package frc.robot;

import java.time.Instant;
import java.util.Date;

//Robot independent functions that could be useful any year
public interface Util {
    public static double deadzoneValue(double input, double deadZone) {

        if (input < -deadZone) {
            return (input + deadZone) / (1 - deadZone);
        } else if (input > deadZone) {
            return (input - deadZone) / (1 - deadZone);
        } else {
            return 0;
        }
    }

    //https://benjiweber.co.uk/blog/2013/12/08/null-coalescing-in-java-8/
    @SafeVarargs public static <T> T coalesce(T... ts) {
        for (T t : ts)
            if (t != null)
                return t;

        return null;
    }

    /* A routine to make angle differences map to a continuous domain [-Pi,Pi].*/
    public static double normalizeAngleRadians(double radians) {
        if (radians > Math.PI) {
            radians -= 2*Math.PI;
        }
        if (radians < -Math.PI) {
            radians += 2*Math.PI;
        }
        return radians;
    }

    public static Date getCompileDate() {
        try {
            return new Date(
                Robot
                    .class
                    .getResource("Main.class")
                    .openConnection()
                    .getLastModified()
            );
        } catch (Exception ignored) {
            return Date.from(Instant.EPOCH);
        }
    }
    public static double speedScale(double startDistance, double endDistance, double startSpeed, double endSpeed, double currentDistance){
        //y = mx + b (math == fun)
        double slope = (endSpeed - startSpeed) / (endDistance - startDistance);
        double currentSpeed = slope * e(currentDistance - startDistance) + startSpeed;

        return currentSpeed;
    }
}
