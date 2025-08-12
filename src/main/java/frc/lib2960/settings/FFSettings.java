package frc.lib2960.settings;



public class FFSettings {

    // TODO Implement using Units library
    public double kS;
    public double kG;
    public double kV;
    public double kA;

    /**
     * Constructor
     * 
     * @param kS Voltage to overcome static friction
     * @param kG Voltage to overcome gravity
     * @param kV Voltage per angular velocity of the system
     * @param kA Voltage per angular acceleration of the system
     */
    public FFSettings(double kS, double kG, double kV, double kA) {
        this.kS = kS;
        this.kG = kG;
        this.kV = kV;
        this.kA = kA;
    }

    /**
     * Constructor
     * 
     * @param kS Voltage to overcome static friction
     * @param kG Voltage to overcome gravity
     * @param kV Voltage per angular velocity of the system
     */
    public FFSettings(double kS, double kG, double kV) {
        this(kS, kG, kV, 0);
    }

    /**
     * Constructor
     * 
     * @param kS Voltage to overcome static friction
     * @param kV Voltage per angular velocity of the system
     */
    public FFSettings(double kS, double kV) {
        this(kS, 0, kV, 0);
    }

}
