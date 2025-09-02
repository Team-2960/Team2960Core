package frc.lib2960.config.device;

/**
 * Common configurations for a motor
 */
public class MotorConfig {
    public final String name;
    public final int id;
    private boolean invert = false;
    private double gearRatio = 1;

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

    /**
     * Gets the inverted flag.
     * @return  inverted flag.
     */
    public boolean getInverted() {
        return invert;
    }

    /**
     * Gets the gear ratio between the motor and the mechanism
     * @return gear ratio between the motor and the mechanism
     */
    public double getGearRatio() {
        return gearRatio;
    }
}
