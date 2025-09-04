package frc.lib2960.config.subsystem;

import static edu.wpi.first.units.Units.Amps;

import java.util.Optional;

import edu.wpi.first.units.measure.Current;
import frc.lib2960.config.device.MotorConfig;
import frc.lib2960.config.device.EncoderConfig;

public class MotorMechCommonConfig {
    public enum LimitTrim{ Voltage, Velocity}

    /** Name of the mechanism */
    public String name;

    /** Array of motor configurations for all the motors in the mechanism. */
    public MotorConfig[] motorConfigs;

    /** Encoder configuration for the mechanism. Set to empty if no encoder is present. Defaults to empty, */
    public Optional<EncoderConfig> encoderConfig = Optional.empty();

    /** Sets the maximum per motor current for the mechanism. Defaults to 80A. */
    public Current maxMotorCurrent = Amps.of(80);

    /** Sets method for keeping mechanism from exceeding its limits if they are set. Defaults to LimitTrim.Voltage.  */
    public LimitTrim limitTrim = LimitTrim.Voltage;

    /**
     * Constructor
     * 
     * @param name            name of the mechanism
     * @param motorConfigs    motor configurations
     */
    public MotorMechCommonConfig(
            String name,
            MotorConfig... motorConfigs) {
        this.name = name;
        this.motorConfigs = motorConfigs;
    }

    /**
     * Set the encoder config. Defaults to empty.
     * @param config    encoder Config
     * @return current configuration object
     */
    public MotorMechCommonConfig setEncoderConfig(EncoderConfig config) {
        this.encoderConfig = Optional.of(config);
        return this;
    }

    /**
     * Clears the encoder configuration.
     * @return current configuration object
     */
    public MotorMechCommonConfig clearEncoderConfig() {
        this.encoderConfig = Optional.empty();
        return this;
    }

    /**
     * Sets the maximum per motor current. Defaults to 80A.
     * @param current   maximum per motor current.
     * @return current configuration object
     */
    public MotorMechCommonConfig setMaxCurrent(Current current) {
        this.maxMotorCurrent = current;
        return this;
    }

    /**
     * Sets the method used to keep the mechanism from exceeding its limits. Defaults to LimitTrim.Voltage.
     * @param limitTrim mechanism limit trimming method
     * @return current configuration object
     */
    public MotorMechCommonConfig setLimitTrim(LimitTrim limitTrim) {
        this.limitTrim = limitTrim;
        return this;
    }
}
