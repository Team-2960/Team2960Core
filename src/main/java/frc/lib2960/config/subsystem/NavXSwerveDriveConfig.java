package frc.lib2960.config.subsystem;

import com.studica.frc.AHRS.NavXComType;

public class NavXSwerveDriveConfig {
    public SwerveDriveBaseConfig baseConfig = new SwerveDriveBaseConfig();
    private NavXComType comType = NavXComType.kMXP_SPI;

    /**
     * Sets the NavX com type. Default is "kMXP_SPI".
     * @param comType NavX com type
     * @return current config object
     */
    public NavXSwerveDriveConfig setComType(NavXComType comType) {
        this.comType = comType;
        return this;
    }

    /**
     * Gets the NavX com type.
     * @return NavX com type.
     */
    public NavXComType getComType() {
        return comType;
    }

}
