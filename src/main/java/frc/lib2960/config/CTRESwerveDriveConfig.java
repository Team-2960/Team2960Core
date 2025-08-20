package frc.lib2960.config;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;

public class CTRESwerveDriveConfig extends SwerveDriveBaseConfig {
    public final int imuCANID;
    public final String CANBusName;
    public final Frequency odometryUpdateFrequency;

    public CTRESwerveDriveConfig(
            LinearControllerConfig linearCtrlConfig,
            AngularControllerConfig angleCtrlConfig,
            Time period,
            Vector<N3> stateStd,
            Vector<N3> visionStd,
            int imuCANID,
            String CANBusName,
            Frequency odometryUpdateFrequency) {
        super(linearCtrlConfig, angleCtrlConfig, period, stateStd, visionStd);

        this.imuCANID = imuCANID;
        this.CANBusName = CANBusName;
        this.odometryUpdateFrequency = odometryUpdateFrequency;
    }
}
