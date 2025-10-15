package frc.lib2960.helper;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class RobotFeature {
    public static final RobotFeature origin = new RobotFeature("origin", Inches.zero(), Inches.zero(), Degrees.zero());

    /** Name of the feature */
    public final String name;

    /** X offset of the robot feature */
    public final Distance x;

    /** Y offset of the robot feature */
    public final Distance y;

    /** Orientation of the robot feature */
    public final Angle r;

    /**
     * Constructor
     * 
     * @param name feature name
     * @param x    X offset of the robot feature
     * @param y    Y offset of the robot feature
     * @param r    Orientation of the robot feature
     */
    public RobotFeature(String name, Distance x, Distance y, Angle r) {
        this.name = name;
        this.x = x;
        this.y = y;
        this.r = r;
    }

    /**
     * Constructor
     * 
     * @param name feature name
     * @param x    X offset of the robot feature
     * @param y    Y offset of the robot feature
     */
    public RobotFeature(String name, Distance x, Distance y) {
        this(name, x, y, Degrees.zero());
    }

    /**
     * Constructor
     * 
     * @param name feature name
     * @param r    Orientation of the robot feature
     */
    public RobotFeature(String name, Angle r) {
        this(name, Inches.zero(), Inches.zero(), r);
    }

    /**
     * Get translation for the feature
     * @return  translation for the feature
     */
    public Translation2d getTranslation() {
        return new Translation2d(x, y);
    }
}
