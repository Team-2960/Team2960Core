package frc.lib2960.config.subsystem;

import edu.wpi.first.units.measure.Frequency;

public class CTRESwerveDriveConfig {
    public final SwerveDriveBaseConfig baseConfig = new SwerveDriveBaseConfig();
    public final int imuCANID;
    public final String CANBusName;
    public final Frequency odometryUpdateFrequency;

    public CTRESwerveDriveConfig(
            int imuCANID,
            String CANBusName,
            Frequency odometryUpdateFrequency) {

        this.imuCANID = imuCANID;
        this.CANBusName = CANBusName;
        this.odometryUpdateFrequency = odometryUpdateFrequency;
    }
}
