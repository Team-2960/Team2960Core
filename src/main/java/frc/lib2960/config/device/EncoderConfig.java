package frc.lib2960.config.device;

/**
 * Common configurations for a encoder
 */
public class EncoderConfig {
    /** Name of the encoder */
    public final String name;

    /** ID of the encoder */
    public final int id;

    /** Inverted flag. Defaults to false. */
    public boolean invert = false;

    /** CAN Bus name */
    public String CANBusName = "Rio";

    /**
     * Gear ratio between the encoder and the mechanism. Should not include final
     * pulley diameter in a linear system. Defaults to 1.
     */
    public double gearRatio = 1;

    /**
     * Constructor
     * 
     * @param name name of the encoder
     * @param id   encoder id
     */
    public EncoderConfig(String name, int id) {
        this.name = name;
        this.id = id;
    }

    /**
     * Constructor. Name defaulted to "Encoder <id>".
     * 
     * @param id encoder id
     */
    public EncoderConfig(int id) {
        this(String.format("Encoder %02d", id), id);
    }

    /**
     * Sets the inverted flag. Default is false.
     * 
     * @param invert inverted flag
     * @return current configuration object
     */
    public EncoderConfig setInverted(boolean invert) {
        this.invert = invert;
        return this;
    }

    /**
     * Sets the CAN bus name. Default is "Rio".
     * 
     * @param CANBusName
     * @return current configuration object
     */
    public EncoderConfig setCANBusName(String CANBusName) {
        this.CANBusName = CANBusName;
        return this;
    }

    /**
     * Sets the gear ratio between the encoder and the mechanism. Final drive pulley
     * for linear mechanism should not be included. Default is 1.
     * 
     * @param gearRatio gear ratio between the encoder and the mechanism
     * @return current configuration object
     */
    public EncoderConfig setGearRatio(double gearRatio) {
        this.gearRatio = gearRatio;
        return this;
    }
}
