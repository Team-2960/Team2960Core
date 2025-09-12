package frc.robot.subsystems;

import edu.wpi.first.units.measure.Voltage;
import frc.lib2960.config.device.MotorConfig;
import frc.lib2960.config.subsystem.AngularMotorMechConfig;

public class AlgaeRollerConfig {
    public AngularMotorMechConfig motorMechConfig;
    public MotorConfig motorConfig;
    public Voltage intakeVolt;
    public Voltage ejectVolt;

    /**
     * Constructor
     * 
     * @param motorMechConfig Motor mechanism configuration
     * @param motorConfig     Motor configuration
     * @param intakeVolt      Intake voltage
     * @param ejectVolt       Eject voltage
     */
    public AlgaeRollerConfig(
            AngularMotorMechConfig motorMechConfig,
            MotorConfig motorConfig,
            Voltage intakeVolt,
            Voltage ejectVolt) {
        this.motorMechConfig = motorMechConfig;
        this.motorConfig = motorConfig;
        this.intakeVolt = intakeVolt;
        this.ejectVolt = ejectVolt;
    }
}
