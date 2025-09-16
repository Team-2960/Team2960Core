package frc.lib2960.config.device;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.Current;

/**
 * Common configurations for a motor
 */
public class MotorConfig {
    /** Name of the motor */
    public final String name;

    /** ID of the motor */
    public final int id;

    /** Inverted flag. Defaults to false. */
    public boolean invert = false;

    /** CAN Bus name */
    public String CANBusName = "Rio";

    /** < Motor gear reduction */
    public double gearRatio = 1;

    /** Sets the maximum per motor current for the mechanism. Defaults to 80A. */
    public Current maxMotorCurrent = Amps.of(80);

    /**
     * Constructor
     * 
     * @param name name of the motor
     * @param id   motor id
     */
    public MotorConfig(String name, int id) {
        this.name = name;
        this.id = id;
    }

    /**
     * Constructor. Name defaulted to "Motor <id>".
     * 
     * @param id motor id
     */
    public MotorConfig(int id) {
        this(String.format("Motor %02d", id), id);
    }

    /**
     * Sets the inverted flag. Default is false.
     * 
     * @param invert inverted flag
     * @return current configuration object
     */
    public MotorConfig setInverted(boolean invert) {
        this.invert = invert;
        return this;
    }

    /**
     * Sets the CAN bus name. Default is "Rio".
     * 
     * @param CANBusName
     * @return current configuration object
     */
    public MotorConfig setCANBusName(String CANBusName) {
        this.CANBusName = CANBusName;
        return this;
    }

    /**
     * Sets the motor gear ratio. Default is 1.
     * 
     * @param gearRatio motor gear ratio
     * @return current configuration object
     */
    public MotorConfig setGearRatio(double gearRatio) {
        this.gearRatio = gearRatio;
        return this;
    }

    /**
     * Sets the maximum per motor current. Defaults to 80A.
     * 
     * @param current maximum per motor current.
     * @return current configuration object
     */
    public MotorConfig setMaxCurrent(Current current) {
        this.maxMotorCurrent = current;
        return this;
    }

}
