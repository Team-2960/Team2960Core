package frc.lib2960.config.subsystem;

import java.util.HashMap;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.lib2960.config.controller.LinearControllerConfig;

public class LinearMotorMechConfig {
    /** Common motor mechanism configrations. */
    public MotorMechCommonConfig common;

    /** Diameter of the output pulley. */
    public Distance pulleyDiameter;
    /** Radius of the output pulley. */
    public Distance pulleyRadius;
    /** Circumfrance of the output pulley. */
    public Distance pulleyCircumfrance;

    /** Motion control configruation */
    public LinearControllerConfig controlConfig = new LinearControllerConfig();

    /** Preset position list */
    public final HashMap<String, Distance> presetPos = new HashMap<>();

    /** Preset velocity list */
    public final HashMap<String, LinearVelocity> presetVel = new HashMap<>();

    /**
     * Constructor
     * 
     * @param name           name of the mechanism
     * @param pulleyDiameter diameter of the output pulley
     * @param motorConfigs   motor configurations
     */
    public LinearMotorMechConfig(
            String name,
            Distance pulleyDiameter) {
        this.pulleyDiameter = pulleyDiameter;
        this.pulleyRadius = pulleyDiameter.div(2);
        this.pulleyCircumfrance = pulleyDiameter.times(Math.PI);
    }

    /**
     * Sets the linear control configuration. Defaults to new
     * LinearControllerConfig().
     * 
     * @param controlConfig LinearControllerConfig object
     * @return current configuration object
     */
    public LinearMotorMechConfig setLinearMotorMechConfig(LinearControllerConfig controlConfig) {
        this.controlConfig = controlConfig;
        return this;
    }

    /**
     * Adds a named preset position.
     * 
     * @param name     name of the preset.
     * @param position target position
     * @return current configuration object
     */
    public LinearMotorMechConfig addPreset(String name, Distance preset) {
        presetPos.put(name, preset);
        return this;
    }

    /**
     * Adds a named preset velocity.
     * 
     * @param name     name of the preset.
     * @param position target position
     * @return current configuration object
     */
    public LinearMotorMechConfig addPreset(String name, LinearVelocity preset) {
        presetVel.put(name, preset);
        return this;
    }
}
