package frc.lib2960.config;

import java.util.Optional;

import edu.wpi.first.units.measure.Current;
import frc.lib2960.config.basic.MotorConfig;
import frc.lib2960.config.basic.EncoderConfig;

public class MotorMechBaseConfig {
    public final String name;

    public final MotorConfig[] motorsConfigs;
    public final Optional<EncoderConfig> encoderConfig;

    public final Optional<Current> maxMotorCurrent;

    /**
     * Constructor
     * 
     * @param name            name of the mechanism
     * @param encoderConfig   encoder configuration
     * @param maxMotorCurrent maximum allowable current per motor
     * @param motorConfigs    motor configurations
     */
    public MotorMechBaseConfig(
            String name,
            Optional<EncoderConfig> encoderConfig,
            Optional<Current> maxMotorCurrent,
            MotorConfig... motorConfigs) {
        this.name = name;
        this.encoderConfig = encoderConfig;
        this.maxMotorCurrent = maxMotorCurrent;
        this.motorsConfigs = motorConfigs;
    }
}
