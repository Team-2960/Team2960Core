package frc.lib2960.config.device;

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
    /**
     * Gear ratio between the motor and the mechanism. Should not include final
     * pulley diameter in a linear system. Defaults to 1.
     */
    public double gearRatio = 1;

    /**
     * Constructor
     * 
     * @param name   name of the motor
     * @param id     motor id
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
     * @param invert    inverted flag
     * @return current configuration object
     */
    public MotorConfig setInverted(boolean invert) {
        this.invert = invert;
        return this;
    }

    /**
     * Sets the gear ratio between the motor and the mechanism. Final drive pulley for linear mechanism should not be included. Default is 1.
     * @param gearRatio gear ratio between the motor and the mechanism
     * @return current configuration object
     */
    public MotorConfig setGearRatio(double gearRatio) {
        this.gearRatio = gearRatio;
        return this;
    }
}
