package frc.lib2960.helper;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;

public class AngleUtil {
    /**
     * Calculates the nearest target angle to the current angle.
     * 
     * @param current current angle. Angle is unwrapped to -180 to 180 degrees.
     * @param target  target angle.Angle is unwrapped to -180 to 180 degrees.
     * @return New MutAngle object with the nearest target angle.
     */
    public static MutAngle Angle(Angle current, Angle target) {
        MutAngle result = Degrees.mutable(0);
        nearestRotation(current, target, result);
        return result;
    }

    /**
     * Calculates the nearest target angle to the current angle.
     * 
     * @param current current angle. Angle is unwrapped to -180 to 180 degrees.
     * @param target  target angle.Angle is unwrapped to -180 to 180 degrees.
     * @param result  Field to store the nearest target angle.
     */
    public static void nearestRotation(Angle current, Angle target, MutAngle result) {
        result.mut_replace(
                nearestRotationDegrees(current.in(Degrees), target.in(Degrees)),
                Degrees);
    }

    /**
     * Calculates the nearest target angle to the current angle.
     * 
     * @param current current angle in degrees. Angle is unwrapped to -180 to 180
     *                degrees.
     * @param target  target angle in degrees. Angle is unwrapped to -180 to 180
     *                degrees.
     * @return The nearest target angle in degrees.
     */
    public static double nearestRotationDegrees(double current, double target) {
        current = unwrapDegrees(current);
        target = unwrapDegrees(target);
        double error = target - current;

        if (Math.abs(error - 360) < Math.abs(error))
            target -= 360;
        if (Math.abs(error + 360) < Math.abs(error))
            target += 360;

        return target;
    }

    /**
     * Calculates the nearest target angle to the current angle.
     * 
     * @param current current angle in radians. Angle is unwrapped to -180 to 180
     *                radians.
     * @param target  target angle in radians. Angle is unwrapped to -180 to 180
     *                radians.
     * @return The nearest target angle in radians.
     */
    public static double nearestRotationRadians(double current, double target) {
        current = unwrapRadians(current);
        target = unwrapRadians(target);
        double error = target - current;

        if (Math.abs(error - Math.PI) < Math.abs(error))
            target -= Math.PI;
        if (Math.abs(error + Math.PI) < Math.abs(error))
            target += Math.PI;

        return target;
    }

    /**
     * Unwraps an supplied angle so it is between -180 and 180 degrees.
     * 
     * @param angle  angle to unwrap
     * @param result field to store the unwrapped value
     */
    public static void unwrapAngle(Angle angle, MutAngle result) {
        result.mut_replace(unwrapDegrees(angle.in(Degrees)), Degrees);
    }

    /**
     * Unwraps an supplied angle so it is between -180 and 180 degrees.
     * 
     * @param angle angle to unwrap in degrees
     * @return unwrapped angle in degrees
     */
    public static double unwrapDegrees(double angle) {
        angle %= 360.0;

        if (angle > 180)
            angle -= 360;
        if (angle < -180)
            angle += 360;

        return angle;
    }

    /**
     * Unwraps an supplied angle so it is between -180 and 180 radians.
     * 
     * @param angle angle to unwrap in radians
     * @return unwrapped angle in radians
     */
    public static double unwrapRadians(double angle) {
        angle %= Math.PI;

        if (angle > Math.PI / 2)
            angle -= Math.PI;
        if (angle < -Math.PI / 2)
            angle += Math.PI;

        return angle;
    }

    /**
     * Checks if an angle is within tolerance of a target in degrees
     * 
     * @param current   current angle in degrees
     * @param target    target angle in degrees
     * @param tolerance tolerance in degrees (inclusive)
     * @return true if in tolerance, false otherwise
     */
    public static boolean inToleranceDegrees(double current, double target, double tolerance) {
        target = nearestRotationDegrees(current, target);
        return current >= target - tolerance && current < target + tolerance;
    }

    /**
     * Checks if an angle is within tolerance of a target in radians
     * 
     * @param current   current angle in radians
     * @param target    target angle in radians
     * @param tolerance tolerance in radians (inclusive)
     * @return true if in tolerance, false otherwise
     */
    public static boolean inToleranceRadians(double current, double target, double tolerance) {
        target = nearestRotationRadians(current, target);
        return current >= target - tolerance && current < target + tolerance;
    }

    /**
     * Converts a Units library angle to a Rotation2d object
     * 
     * @param angle Units library angle
     * @return new Rotation2d object
     */
    public static Rotation2d toR2d(Angle angle) {
        return Rotation2d.fromDegrees(angle.in(Degrees));
    }

    /**
     * Converts a Rotation2d object to a Units library Angle
     * 
     * @param angle Rotation2d object
     * @return new Units library object
     */
    public static Angle toUnits(Rotation2d angle) {
        return Degrees.of(angle.getDegrees());
    }

    /**
     * Converts a Rotation2d object to a Units library angle
     * 
     * @param angle  Rotation2d object
     * @param result Object to store the result in
     */
    public static void toUnits(Rotation2d angle, MutAngle result) {
        result.mut_replace(angle.getDegrees(), Degrees);
    }
}
