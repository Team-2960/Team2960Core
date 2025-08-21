package frc.lib2960.helper;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;

public class MutVector2d<U extends Unit> {
    public final MutableMeasure<U, ?, ?> x; /* < X component */
    public final MutableMeasure<U, ?, ?> y; /* < y component */

    
    /**
     * Constructor
     * 
     * @param x x component
     * @param y y component
     */
    public MutVector2d(Measure<U> x, Measure<U> y) {
        this.x = x.mutableCopy();
        this.y = y.mutableCopy();
    }

    /**
     * Constructor
     * 
     * @param m magnitude
     * @param a angle
     */
    public MutVector2d(Measure<U> m, Angle a) {
        x = m.mutableCopy();
        x.mut_setMagnitude(m.magnitude() * Math.cos(a.in(Radians)));

        y = m.mutableCopy();
        y.mut_setMagnitude(m.magnitude() * Math.sin(a.in(Radians)));
    }

    /**
     * Calculates the magnitude
     * @param result mutable measure to store the magnitude
     */
    public void magnitude(MutableMeasure<U, ?, ?> result) {
        double x_mag = x.magnitude();
        double y_mag = y.magnitude();

        result.mut_setMagnitude(Math.sqrt(x_mag * x_mag + y_mag * y_mag));
    }

    /**
     * Calculates the angle
     * @param result mutable measure to store the angle
     */
    public void angle(MutAngle result) {
        double x_mag = x.magnitude();
        double y_mag = y.magnitude();

        result.mut_replace(Math.atan2(y_mag, x_mag), Radians);
    }

}
