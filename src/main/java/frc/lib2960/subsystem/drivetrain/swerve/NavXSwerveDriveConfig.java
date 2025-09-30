package frc.lib2960.subsystem.drivetrain.swerve;

import com.studica.frc.AHRS.NavXComType;

public class NavXSwerveDriveConfig {
    public SwerveDriveCommonConfig common = new SwerveDriveCommonConfig();
    public NavXComType comType = NavXComType.kMXP_SPI;

    /**
     * Sets the common config. Defaults to new SwerveDriveCommonConfig().
     * 
     * @param common new common config
     * @return current config object
     */
    public NavXSwerveDriveConfig setCommonConfig(SwerveDriveCommonConfig common) {
        this.common = common;
        return this;
    }

    /**
     * Sets the NavX com type. Default is "kMXP_SPI".
     * 
     * @param comType NavX com type
     * @return current config object
     */
    public NavXSwerveDriveConfig setComType(NavXComType comType) {
        this.comType = comType;
        return this;
    }
}
