package frc.lib2960.subsystem.motor;

import frc.lib2960.subsystem.motor.SparkMotorMechConfig.PosEncoderSource;
import frc.lib2960.subsystem.motor.SparkMotorMechConfig.VelEncoderSource;

public class AngularSparkMechConfig {
    /** Common Angular Mechanism Config */
    public AngularMotorMechConfig commonConfig;

    /** Position encoder source */
    public PosEncoderSource posEncoderSource = PosEncoderSource.INTERNAL;

    /** Velocity encoder source */
    public VelEncoderSource velEncoderSource = VelEncoderSource.INTERNAL;

    /** Position Encoder Gear */
    public double posEncoderGearRatio = 1;

    /** Velocity Encoder Gear */
    public double velEncoderGearRatio = 1;

    /**
     * Constructor
     * 
     * @param name mechanism name
     */
    public AngularSparkMechConfig(String name) {
        this.commonConfig = new AngularMotorMechConfig(name);
    }

    /**
     * Constructor
     * 
     * @param commonConfig Common Angular Mechanism Config
     */
    public AngularSparkMechConfig(AngularMotorMechConfig commonConfig) {
        this.commonConfig = commonConfig;
    }

    /**
     * Sets the common angular motor mech config
     * 
     * @param commonConfig common angular motor mech config
     * @return current configuration object
     */
    public AngularSparkMechConfig setMotorMechConfig(AngularMotorMechConfig commonConfig) {
        this.commonConfig = commonConfig;
        return this;
    }

    /**
     * Sets the position encoder source
     * 
     * @param source position encoder source. Defaults to INTERNAL.
     * @return current configuration object
     */
    public AngularSparkMechConfig setPosEncoderSource(PosEncoderSource source) {
        posEncoderSource = source;
        return this;
    }

    /**
     * Sets the velocity encoder source
     * 
     * @param source velocity encoder source. Defaults to INTERNAL.
     * @return current configuration object
     */
    public AngularSparkMechConfig setVelEncoderSource(VelEncoderSource source) {
        velEncoderSource = source;
        return this;
    }

    /**
     * Sets both encoder type gear ratios
     * 
     * @param gearRatio encoder gear ratio. Both default to 1.
     * @return current configuration object
     */
    public AngularSparkMechConfig setEncoderGearRatio(double gearRatio) {
        posEncoderGearRatio = gearRatio;
        velEncoderGearRatio = gearRatio;
        return this;
    }

    /**
     * Sets position encoder gear ratios
     * 
     * @param gearRatio position encoder gear ratio. Both default to 1.
     * @return current configuration object
     */
    public AngularSparkMechConfig setPosEncoderGearRatio(double gearRatio) {
        posEncoderGearRatio = gearRatio;
        return this;
    }

    /**
     * Sets velocity encoder gear ratios
     * 
     * @param gearRatio velocity encoder gear ratio. Both default to 1.
     * @return current configuration object
     */
    public AngularSparkMechConfig setVelEncoderGearRatio(double gearRatio) {
        velEncoderGearRatio = gearRatio;
        return this;
    }
}
