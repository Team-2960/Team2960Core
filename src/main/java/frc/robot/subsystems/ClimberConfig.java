package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.lib2960.config.device.MotorConfig;
import frc.lib2960.config.subsystem.AngularMotorMechConfig;

public class ClimberConfig {
    public AngularMotorMechConfig motorMechConfig;
    public MotorConfig motorConfig;
    public Voltage extendVolt;
    public Voltage retractVolt;
    public Angle extendAngle;
    public Angle climbAngle;
    public Angle homeAngle;

    /**
     * Constructor
     * 
     * @param motorMechConfig Motor mechanism configuration
     * @param motorConfig     Motor configuration
     * @param extendVolt      Motor voltage for extending the climber
     * @param retractVolt     Motor voltage for retracting the climber
     * @param extendAngle     Angle to extend the climber
     * @param climbAngle      Angle to climb the climber
     * @param homeAngle       Home position of the climber
     */
    public ClimberConfig(
            AngularMotorMechConfig motorMechConfig,
            MotorConfig motorConfig,
            Voltage extendVolt,
            Voltage retractVolt,
            Angle extendAngle,
            Angle climbAngle,
            Angle homeAngle,
            Angle tolerance) {
        this.motorMechConfig = motorMechConfig;
        this.motorConfig = motorConfig;
        this.extendVolt = extendVolt;
        this.retractVolt = retractVolt;
        this.extendAngle = extendAngle;
        this.climbAngle = climbAngle;
        this.homeAngle = homeAngle;
    }
}
