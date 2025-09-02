package frc.lib2960.config.subsystem;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Revolutions;

import java.util.Optional;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Per;
import frc.lib2960.config.device.MotorConfig;
import frc.lib2960.config.controller.LinearControllerConfig;
import frc.lib2960.config.device.EncoderConfig;

public class LinearMotorMechConfig extends MotorMechBaseConfig {
    public final Per<DistanceUnit, AngleUnit> motorToMechRatio;
    public final Per<DistanceUnit, AngleUnit> encToMechRatio;

    public final Optional<LinearControllerConfig> controlConfig;

    /**
     * Constructor
     * 
     * @param name             name of the mechanism
     * @param encoderConfig    encoder configuration
     * @param maxMotorCurrent  maximum allowable current per motor
     * @param motorToMechRatio ratio between linear motion of the mechanism to
     *                         rotation of the motor(s)
     * @param encToMechRatio   ratio between linear motion of the mechanism to
     *                         rotation of the encoder
     * @param motorConfigs     motor configurations
     */
    public LinearMotorMechConfig(
            String name,
            EncoderConfig encoderConfig,
            Current maxMotorCurrent,
            Per<DistanceUnit, AngleUnit> motorToMechRatio,
            Per<DistanceUnit, AngleUnit> encToMechRatio,
            LinearControllerConfig controlConfig,
            MotorConfig... motorConfigs) {
        super(name, Optional.of(encoderConfig), Optional.of(maxMotorCurrent), motorConfigs);
        this.motorToMechRatio = motorToMechRatio;
        this.encToMechRatio = encToMechRatio;
        this.controlConfig = Optional.of(controlConfig);
    }

    /**
     * Constructor
     * 
     * @param name             name of the mechanism
     * @param maxMotorCurrent  maximum allowable current per motor
     * @param motorToMechRatio ratio between linear motion of the mechanism to
     *                         rotation of the motor(s)
     * @param motorConfigs     motor configurations
     */
    public LinearMotorMechConfig(
            String name,
            Current maxMotorCurrent,
            Per<DistanceUnit, AngleUnit> motorToMechRatio,
            MotorConfig... motorConfigs) {
        super(name, Optional.empty(), Optional.of(maxMotorCurrent), motorConfigs);
        this.motorToMechRatio = motorToMechRatio;
        this.encToMechRatio = Meters.per(Revolutions).ofNative(1);
        this.controlConfig = Optional.empty();
    }

    /**
     * Constructor
     * 
     * @param name             name of the mechanism
     * @param encoderConfig    encoder configuration
     * @param motorToMechRatio ratio between linear motion of the mechanism to
     *                         rotation of the motor(s)
     * @param encToMechRatio   ratio between linear motion of the mechanism to
     *                         rotation of the encoder
     * @param motorConfigs     motor configurations
     */
    public LinearMotorMechConfig(
            String name,
            EncoderConfig encoderConfig,
            Per<DistanceUnit, AngleUnit> motorToMechRatio,
            Per<DistanceUnit, AngleUnit> encToMechRatio,
            LinearControllerConfig controlConfig,
            MotorConfig... motorConfigs) {
        super(name, Optional.of(encoderConfig), Optional.empty(), motorConfigs);
        this.motorToMechRatio = motorToMechRatio;
        this.encToMechRatio = encToMechRatio;
        this.controlConfig = Optional.of(controlConfig);
    }

    /**
     * Constructor
     * 
     * @param name             name of the mechanism
     * @param motorToMechRatio ratio between linear motion of the mechanism to
     *                         rotation of the motor(s)
     * @param motorConfigs     motor configurations
     */
    public LinearMotorMechConfig(
            String name,
            Per<DistanceUnit, AngleUnit> motorToMechRatio,
            MotorConfig... motorConfigs) {
        super(name, Optional.empty(), Optional.empty(), motorConfigs);
        this.motorToMechRatio = motorToMechRatio;
        this.encToMechRatio = Meters.per(Revolutions).ofNative(1);
        this.controlConfig = Optional.empty();
    }
}
