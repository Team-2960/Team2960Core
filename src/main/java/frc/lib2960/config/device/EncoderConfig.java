package frc.lib2960.config.device;

/**
 * Common configurations for a encoder
 */
public class EncoderConfig {
    public final String name;
    public final int id;
    private boolean invert = false;
    private double gearRatio = 1;

    /**
     * Constructor
     * 
     * @param name   name of the encoder
     * @param id     encoder id
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
     * @param invert    inverted flag
     * @return current configuration object
     */
    public EncoderConfig setInverted(boolean invert) {
        this.invert = invert;
        return this;
    }

    /**
     * Sets the gear ratio between the encoder and the mechanism. Final drive pulley for linear mechanism should not be included. Default is 1.
     * @param gearRatio gear ratio between the encoder and the mechanism
     * @return current configuration object
     */
    public EncoderConfig setGearRatio(double gearRatio) {
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
     * Gets the gear ratio between the encoder and the mechanism
     * @return gear ratio between the encoder and the mechanism
     */
    public double getGearRatio() {
        return gearRatio;
    }
}
