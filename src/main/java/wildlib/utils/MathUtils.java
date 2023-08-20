package wildlib.utils;

public class MathUtils {
    public static boolean closeEnough(double input, double target, double dif) {
        return input >= target - dif && input <= target + dif;
    }
}
