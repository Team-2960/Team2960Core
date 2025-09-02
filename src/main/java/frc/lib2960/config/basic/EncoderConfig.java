package frc.lib2960.config.basic;

/**
 * Common configurations for a encoder
 */
public class EncoderConfig {
    public final String name;
    public final int id;
    public final boolean invert;

    /**
     * Constructor
     * 
     * @param name   name of the encoder
     * @param id     encoder id
     * @param invert true to invert the encoder, false otherwise
     */
    public EncoderConfig(String name, int id, boolean invert) {
        this.name = name;
        this.id = id;
        this.invert = invert;
    }

    /**
     * Constructor. Name defaulted to "Encoder <id>".
     * 
     * @param id     encoder id
     * @param invert true to invert the encoder, false otherwise
     */
    public EncoderConfig(int id, boolean invert) {
        this(String.format("Encoder %02d", id), id, invert);
    }

    /**
     * Constructor. Inverted set to false.
     * 
     * @param name name of the encoder
     * @param id   encoder id
     */
    public EncoderConfig(String name, int id) {
        this(name, id, false);
    }

    /**
     * Constructor. Name defaulted to "Encoder <id>". Inverted set to false.
     * 
     * @param id encoder id
     */
    public EncoderConfig(int id) {
        this(String.format("Encoder %02d", id), id, false);
    }
}
