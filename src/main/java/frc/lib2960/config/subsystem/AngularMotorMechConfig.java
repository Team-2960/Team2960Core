package frc.lib2960.config.subsystem;

import frc.lib2960.config.device.MotorConfig;
import frc.lib2960.config.controller.AngularControllerConfig;

public class AngularMotorMechConfig {
    /** Common motor mechanism configrations. */
    public MotorMechCommonConfig common;

    /** Motion control configruation */
    public AngularControllerConfig controlConfig = new AngularControllerConfig();

    /**
     * Constructor
     * 
     * @param name           name of the mechanism
     * @param pulleyDiameter diameter of the output pulley
     * @param motorConfigs   motor configurations
     */
    public AngularMotorMechConfig(
            String name,
            MotorConfig... motorConfigs) {
        this.common = new MotorMechCommonConfig(name, motorConfigs);
    }
}
