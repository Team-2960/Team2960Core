package frc.lib2960.config.subsystem;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.Current;

public class MotorMechCommonConfig {
    public enum LimitTrim{ Voltage, Velocity}

    /** Name of the mechanism */
    public String name;

    /** Sets the maximum per motor current for the mechanism. Defaults to 80A. */
    public Current maxMotorCurrent = Amps.of(80);

    /** Sets method for keeping mechanism from exceeding its limits if they are set. Defaults to LimitTrim.Voltage.  */
    public LimitTrim limitTrim = LimitTrim.Voltage;

    /**
     * Constructor
     * 
     * @param name            name of the mechanism
     */
    public MotorMechCommonConfig(String name) {
        this.name = name;
    }

    /**
     * Sets the maximum per motor current. Defaults to 80A.
     * @param current   maximum per motor current.
     * @return current configuration object
     */
    public MotorMechCommonConfig setMaxCurrent(Current current) {
        this.maxMotorCurrent = current;
        return this;
    }

    /**
     * Sets the method used to keep the mechanism from exceeding its limits. Defaults to LimitTrim.Voltage.
     * @param limitTrim mechanism limit trimming method
     * @return current configuration object
     */
    public MotorMechCommonConfig setLimitTrim(LimitTrim limitTrim) {
        this.limitTrim = limitTrim;
        return this;
    }
}
