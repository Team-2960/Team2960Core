package frc.lib2960.config.subsystem;

import java.util.HashMap;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib2960.config.controller.AngularControllerConfig;

public class AngularMotorMechConfig {
    /** Common motor mechanism configurations. */
    public MotorMechCommonConfig common;

    /** Motion control configuration */
    public AngularControllerConfig controlConfig = new AngularControllerConfig();

    /** Preset position list */
    public final HashMap<String, Angle> presetPos = new HashMap<>();

    /** Preset velocity list */
    public final HashMap<String, AngularVelocity> presetVel = new HashMap<>();

    /**
     * Constructor
     * 
     * @param name           name of the mechanism
     * @param pulleyDiameter diameter of the output pulley
     * @param motorConfigs   motor configurations
     */
    public AngularMotorMechConfig(String name) {
        common = new MotorMechCommonConfig(name);
    }

    /**
     * Sets the angular control configuration. Defaults to new
     * AngularControllerConfig().
     * 
     * @param controlConfig AngularControllerConfig object
     * @return current configuration object
     */
    public AngularMotorMechConfig setAngularMotorMechConfig(AngularControllerConfig controlConfig) {
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
    public AngularMotorMechConfig addPreset(String name, Angle preset) {
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
    public AngularMotorMechConfig addPreset(String name, AngularVelocity preset) {
        presetVel.put(name, preset);
        return this;
    }
}
