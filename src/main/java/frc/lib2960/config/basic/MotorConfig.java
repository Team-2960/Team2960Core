package frc.lib2960.config.basic;

/**
 * Common configurations for a motor
 */
public class MotorConfig {
    public final String name;
    public final int id;
    public final boolean invert;

    /**
     * Constructor
     * 
     * @param name   name of the motor
     * @param id     motor id
     * @param invert true to invert the motor, false otherwise
     */
    public MotorConfig(String name, int id, boolean invert) {
        this.name = name;
        this.id = id;
        this.invert = invert;
    }

    /**
     * Constructor. Name defaulted to "Motor <id>".
     * 
     * @param id     motor id
     * @param invert true to invert the motor, false otherwise
     */
    public MotorConfig(int id, boolean invert) {
        this(String.format("Motor %02d", id), id, invert);
    }

    /**
     * Constructor. Inverted set to false.
     * 
     * @param name   name of the motor
     * @param id     motor id
     */
    public MotorConfig(String name, int id) {
        this(name, id, false);
    }

    /**
     * Constructor. Name defaulted to "Motor <id>". Inverted set to false.
     * 
     * @param id     motor id
     */
    public MotorConfig(int id) {
        this(String.format("Motor %02d", id), id, false);
    }
}
