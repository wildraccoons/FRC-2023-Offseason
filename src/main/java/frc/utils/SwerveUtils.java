package frc.utils;

/** 
 * Utility math functions for swerve drive calculations. 
 * Rewrite of the MAXSwerve Java example <a href="https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/utils/SwerveUtils.java"> SwerveUtils.java</a>.
 */
public class SwerveUtils {
    private static final double twoPi = 2*Math.PI;
    
    /**
     * Steps the current value towards the target with a maximum step size.
     * @param current The current (signed) value.
     * @param target The target (signed) that the function will step towards.
     * @param stepSize the maximum step size that can be taken.
     * @return The new value for {@code current} after stepping towards the target.
     */
    public static double StepTowards(double current, double target, double stepSize) {
        final double unsignedStep = Math.abs(stepSize);
        if (Math.abs(current - target) <= unsignedStep) {
            return target;
        } else if (target < current) {
            return current - unsignedStep;
        } else {
            return current + unsignedStep;
        }
    }
    
    /**
     * Steps a value (angle) towards a target (angle) taking the shortest path around a wrapping circle with a maximum step size.
     * @param current The current angle (in radians). Can lie outside the 0-2π range.
     * @param target The target angle (in radians) the function will step towards. Can lie outside the 0-2π range.
     * @param stepSize The maximum step size that can be taken (in radians).
     * @return The new angle (in radians) for {@code current} after stepping towards the target.
     * This value will always lie in the range 0-2π (exclusive).
     */
    public static double StepTowardsCircular(double current, double target, double stepSize) {
        current = WrapAngle(current);
        target = WrapAngle(target);

        double direction = Math.signum(target - current);
        double difference = Math.abs(current - target);

        if (difference <= stepSize) {
            return target;
        } else if (difference > Math.PI) {
            // Hangle case where you can reach the target with wrapping.
            if (current + twoPi - target < stepSize || target + twoPi - current < stepSize) { 
                return target;
            } else {
                return WrapAngle(current - direction * stepSize);
            }
        } else {
            return current + direction * stepSize;
        }
    }
    
    /**
     * Finds the (unsigned) minimum difference between two angles including calculating across 0.
     * @param angleA An angle (in radians).
     * @param angleB An angle (in radians).
     * @return The (unsigned) minimum difference between the two angles (in radians)
     */
    public static double AngleDifference(double angleA, double angleB) {
        double difference = Math.abs(angleA - angleB);
        return difference > Math.PI? twoPi - difference : difference;
    }

    /** 
     * Wraps the given angle (radians) around a circle. I.E. both 2*PI and -2*PI wrap to 0.
     * 
     * @return The wrapped angle. Always within the range 0 to 2*PI (exclusive)
     */
    public static double WrapAngle(double angle) {
        final double inRange = angle % twoPi;

        if (inRange < -0) {
            return twoPi + inRange;
        } else {
            return inRange;
        }
    }
}
