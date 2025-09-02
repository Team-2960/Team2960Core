package frc.lib2960.config.subsystem;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import frc.lib2960.config.controller.AngularControllerConfig;
import frc.lib2960.config.controller.LinearControllerConfig;
import frc.lib2960.config.controller.PIDConfig;

public class CTRESwerveDriveConfig extends SwerveDriveBaseConfig {
    public final int imuCANID;
    public final String CANBusName;
    public final Frequency odometryUpdateFrequency;

    public CTRESwerveDriveConfig(
            String name, 
            LinearControllerConfig linearCtrlConfig,
            AngularControllerConfig angleCtrlConfig,
            Time period,
            PIDConfig linearPPPID,
            PIDConfig angularPPPID,
            int imuCANID,
            String CANBusName,
            Frequency odometryUpdateFrequency) {
        super(name, linearCtrlConfig, angleCtrlConfig, linearPPPID, angularPPPID, period);

        this.imuCANID = imuCANID;
        this.CANBusName = CANBusName;
        this.odometryUpdateFrequency = odometryUpdateFrequency;
    }

    public CTRESwerveDriveConfig(
            String name,
            LinearControllerConfig linearCtrlConfig,
            AngularControllerConfig angleCtrlConfig,
            Time period,
            PIDConfig linearPPPID,
            PIDConfig angularPPPID,
            Vector<N3> stateStd,
            Vector<N3> visionStd,
            int imuCANID,
            String CANBusName,
            Frequency odometryUpdateFrequency) {
        super(name, linearCtrlConfig, angleCtrlConfig, linearPPPID, angularPPPID, period, stateStd, visionStd);

        this.imuCANID = imuCANID;
        this.CANBusName = CANBusName;
        this.odometryUpdateFrequency = odometryUpdateFrequency;
    }
}
