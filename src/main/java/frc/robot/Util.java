package frc.robot;

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
    @SafeVarargs
    public static <T> T coalesce(T... ts) {
        for (T t : ts)
            if (t != null)
                return t;

        return null;
    }

}
