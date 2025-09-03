package frc.lib2960.config.subsystem;

import static edu.wpi.first.units.Units.Hertz;

import edu.wpi.first.units.measure.Frequency;

public class CTRESwerveDriveConfig {
    /** Common Swerve Drive config */
    public SwerveDriveCommonConfig common = new SwerveDriveCommonConfig();
    /** CAN ID of the IMU. */
    public int imuCANID;
    /** CAN Bus Name. Default to "canivore". */
    public String CANBusName = "canivore";
    /** Odometry update frequency. Defaults to 250 hz. */
    public Frequency odometryUpdateFrequency = Hertz.of(250);

    /**
     * Constructor
     * @param imuCANID  IMU CAN ID
     */
    public CTRESwerveDriveConfig(int imuCANID) {
        this.imuCANID = imuCANID;
    }

    /**
     * Sets the CAN Bus Name. Defaults to "canivore".
     * @param CANBusName    CAN Bus Name
     * @return  Current config object
     */
    public CTRESwerveDriveConfig setCANBusName(String CANBusName) {
        this.CANBusName = CANBusName;
        return this;
    }

    /**
     * Sets the odometry update frequency. Defaults to 250 hz.
     * @param freq  odometry update frequency
     * @return  Current config object
     */
    public CTRESwerveDriveConfig setOdometryUpdateFrequency(Frequency freq) {
        this.odometryUpdateFrequency = freq;
        return this;
    }
}
