package frc.lib2960.config.subsystem;

import edu.wpi.first.units.measure.Distance;
import frc.lib2960.config.device.MotorConfig;
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

    /**
     * Constructor
     * 
     * @param name           name of the mechanism
     * @param pulleyDiameter diameter of the output pulley
     * @param motorConfigs   motor configurations
     */
    public LinearMotorMechConfig(
            String name,
            Distance pulleyDiameter,
            MotorConfig... motorConfigs) {
        this.common = new MotorMechCommonConfig(name, motorConfigs);
        this.pulleyDiameter = pulleyDiameter;
        this.pulleyRadius = pulleyDiameter.div(2);
        this.pulleyCircumfrance = pulleyDiameter.times(Math.PI);
    }
}
