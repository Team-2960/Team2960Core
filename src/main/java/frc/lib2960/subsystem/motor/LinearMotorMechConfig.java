package frc.lib2960.subsystem.motor;

import java.util.HashMap;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib2960.config.controller.LinearControllerConfig;
import frc.lib2960.helper.LimitTrim;

public class LinearMotorMechConfig {

    /** Name of the mechanism */
    public String name;

    /** Name of the tab the mechanism will be displayed on */
    public String uiTabName = "Mechanisms";

    /** Distance traveled per revolution of the motor */
    public Distance distPerRev;

    /**
     * Sets method for keeping mechanism from exceeding its limits if they are set.
     * Defaults to LimitTrim.Voltage.
     */
    public LimitTrim limitTrim = LimitTrim.Voltage;

    /** Motion control configruation */
    public LinearControllerConfig controlConfig = new LinearControllerConfig();

    /** Preset position list */
    public final HashMap<String, Distance> presetPos = new HashMap<>();

    /** Preset velocity list */
    public final HashMap<String, LinearVelocity> presetVel = new HashMap<>();

    /** Preset voltage list */
    public final HashMap<String, Voltage> presetVolt = new HashMap<>();

    /**
     * Constructor
     * 
     * @param name           name of the mechanism
     * @param pulleyDiameter diameter of the output pulley
     */
    public LinearMotorMechConfig(
            String name,
            Distance distPerRev) {
        this.name = name;
        this.distPerRev = distPerRev;
    }

    /**
     * Sets the name of the tab the telemetry of this mechanism will appears on.
     * Defaults to "Mechanisms".
     * 
     * @param uiTabName name of the tab the telemetry of this mechanism will appears
     *                  on
     * @return current configuration object
     */
    public LinearMotorMechConfig setUITabName(String uiTabName) {
        this.uiTabName = uiTabName;
        return this;
    }

    /**
     * Sets the method used to keep the mechanism from exceeding its limits.
     * Defaults to LimitTrim.Voltage.
     * 
     * @param limitTrim mechanism limit trimming method
     * @return current configuration object
     */
    public LinearMotorMechConfig setLimitTrim(LimitTrim limitTrim) {
        this.limitTrim = limitTrim;
        return this;
    }

    /**
     * Sets the linear control configuration. Defaults to new
     * LinearControllerConfig().
     * 
     * @param controlConfig LinearControllerConfig object
     * @return current configuration object
     */
    public LinearMotorMechConfig setController(LinearControllerConfig controlConfig) {
        this.controlConfig = controlConfig;
        return this;
    }

    /**
     * Adds a named preset position.
     * 
     * @param name   name of the preset.
     * @param preset target position
     * @return current configuration object
     */
    public LinearMotorMechConfig addPreset(String name, Distance preset) {
        presetPos.put(name, preset);
        return this;
    }

    /**
     * Adds a named preset velocity.
     * 
     * @param name   name of the preset.
     * @param preset target velocity
     * @return current configuration object
     */
    public LinearMotorMechConfig addPreset(String name, LinearVelocity preset) {
        presetVel.put(name, preset);
        return this;
    }

    /**
     * Adds a named preset voltage.
     * 
     * @param name   name of the preset.
     * @param preset target voltage
     * @return current configuration object
     */
    public LinearMotorMechConfig addPreset(String name, Voltage preset) {
        presetVolt.put(name, preset);
        return this;
    }
}
