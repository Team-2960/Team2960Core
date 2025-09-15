package frc.lib2960.config.subsystem;

import static edu.wpi.first.units.Units.Amps;

import java.util.HashMap;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.lib2960.config.controller.AngularControllerConfig;
import frc.lib2960.helper.LimitTrim;

public class AngularMotorMechConfig {

    /** Name of the mechanism */
    public String name;

    /** Name of the tab the mechanism will be displayed on */
    public String uiTabName = "Mechanisms";

    /** Sets the maximum per motor current for the mechanism. Defaults to 80A. */
    public Current maxMotorCurrent = Amps.of(80);

    /**
     * Sets method for keeping mechanism from exceeding its limits if they are set.
     * Defaults to LimitTrim.Voltage.
     */
    public LimitTrim limitTrim = LimitTrim.Voltage;

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
        this.name = name;
    }

    /**
     * Sets the name of the tab the telemetry of this mechanism will appears on.
     * Defaults to "Mechanisms".
     * 
     * @param uiTabName name of the tab the telemetry of this mechanism will appears
     *                  on
     * @return current configuration object
     */
    public AngularMotorMechConfig setUITabName(String uiTabName) {
        this.uiTabName = uiTabName;
        return this;
    }

    /**
     * Sets the maximum per motor current. Defaults to 80A.
     * 
     * @param current maximum per motor current.
     * @return current configuration object
     */
    public AngularMotorMechConfig setMaxCurrent(Current current) {
        this.maxMotorCurrent = current;
        return this;
    }

    /**
     * Sets the method used to keep the mechanism from exceeding its limits.
     * Defaults to LimitTrim.Voltage.
     * 
     * @param limitTrim mechanism limit trimming method
     * @return current configuration object
     */
    public AngularMotorMechConfig setLimitTrim(LimitTrim limitTrim) {
        this.limitTrim = limitTrim;
        return this;
    }

    /**
     * Sets the angular control configuration. Defaults to new
     * AngularControllerConfig().
     * 
     * @param controlConfig AngularControllerConfig object
     * @return current configuration object
     */
    public AngularMotorMechConfig setAngularControllerConfig(AngularControllerConfig controlConfig) {
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
