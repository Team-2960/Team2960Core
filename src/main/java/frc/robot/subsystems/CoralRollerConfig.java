package frc.robot.subsystems;

import edu.wpi.first.units.measure.Voltage;
import frc.lib2960.config.device.MotorConfig;
import frc.lib2960.subsystem.motor.LinearMotorMechConfig;

public class CoralRollerConfig {
    public LinearMotorMechConfig motorMechConfig;
    public MotorConfig motorConfig;
    public int intakeSensorID;
    public Voltage intakeVolt;
    public Voltage ejectVolt;
    public Voltage reverseVolt;

    /**
     * Constructor
     * 
     * @param motorMechConfig Motor mechanism configuration
     * @param motorConfig     Motor configuration
     * @param intakeSensorID  Intake sensor digital input id
     * @param intakeVolt      Intake voltage
     * @param ejectVolt       Eject voltage
     * @param reverseVolt     Reverse voltage
     */
    public CoralRollerConfig(
            LinearMotorMechConfig motorMechConfig,
            MotorConfig motorConfig,
            int intakeSensorID,
            Voltage intakeVolt,
            Voltage ejectVolt,
            Voltage reverseVolt) {
        this.motorMechConfig = motorMechConfig;
        this.motorConfig = motorConfig;
        this.intakeSensorID = intakeSensorID;
        this.intakeVolt = intakeVolt;
        this.ejectVolt = ejectVolt;
        this.reverseVolt = reverseVolt;
    }
}
