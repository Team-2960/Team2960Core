package frc.lib2960.subsystem.motor;

import edu.wpi.first.units.measure.Distance;
import frc.lib2960.subsystem.motor.SparkMotorMechConfig.PosEncoderSource;
import frc.lib2960.subsystem.motor.SparkMotorMechConfig.VelEncoderSource;

public class LinearSparkMechConfig {
    /** Common Linear Mechanism Config */
    public LinearMotorMechConfig commonConfig;

    /** Position encoder source */
    public PosEncoderSource posEncoderSource = PosEncoderSource.INTERNAL;

    /** Velocity encoder source */
    public VelEncoderSource velEncoderSource = VelEncoderSource.INTERNAL;

    /** Position Encoder Gear */
    public Distance posEncoderDistPerRev;

    /** Velocity Encoder Gear */
    public Distance velEncoderDistPerRev;

    /**
     * Constructor
     * 
     * @param name mechanism name
     */
    public LinearSparkMechConfig(String name, Distance distPerRev) {
        this.commonConfig = new LinearMotorMechConfig(name, distPerRev);
        posEncoderDistPerRev = distPerRev;
        velEncoderDistPerRev = distPerRev;
    }

    /**
     * Constructor
     * 
     * @param commonConfig Common Linear Mechanism Config
     */
    public LinearSparkMechConfig(LinearMotorMechConfig commonConfig) {
        this.commonConfig = commonConfig;
        posEncoderDistPerRev = commonConfig.distPerRev;
        velEncoderDistPerRev = commonConfig.distPerRev;
    }

    /**
     * Sets the common linear motor mech config
     * 
     * @param commonConfig common linear motor mech config
     * @return current configuration object
     */
    public LinearSparkMechConfig setMotorMechConfig(LinearMotorMechConfig commonConfig) {
        this.commonConfig = commonConfig;
        return this;
    }

    /**
     * Sets the position encoder source
     * 
     * @param source position encoder source. Defaults to INTERNAL.
     * @return current configuration object
     */
    public LinearSparkMechConfig setPosEncoderSource(PosEncoderSource source) {
        posEncoderSource = source;
        return this;
    }

    /**
     * Sets the velocity encoder source
     * 
     * @param source velocity encoder source. Defaults to INTERNAL.
     * @return current configuration object
     */
    public LinearSparkMechConfig setVelEncoderSource(VelEncoderSource source) {
        velEncoderSource = source;
        return this;
    }

    /**
     * Sets both encoder type distance per revolution
     * 
     * @param distPerRev encoder distance per revolution. Both default to motor distance per rev.
     * @return current configuration object
     */
    public LinearSparkMechConfig setEncoderDistPerRev(Distance distPerRev) {
        posEncoderDistPerRev = distPerRev;
        velEncoderDistPerRev = distPerRev;
        return this;
    }

    /**
     * Sets position encoder distance per revolution
     * 
     * @param distPerRev position encoder distance per revolution. Defaults to motor distance per rev.
     * @return current configuration object
     */
    public LinearSparkMechConfig setPosEncoderDistPerRev(Distance distPerRev) {
        posEncoderDistPerRev = distPerRev;
        return this;
    }

    /**
     * Sets velocity encoder distance per revolution
     * 
     * @param distPerRev velocity encoder distance per revolution. Defaults to motor distance per rev.
     * @return current configuration object
     */
    public LinearSparkMechConfig setVelEncoderDistPerRev(Distance distPerRev) {
        velEncoderDistPerRev = distPerRev;
        return this;
    }
}
