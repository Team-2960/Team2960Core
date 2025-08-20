package frc.lib2960.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib2960.config.NavXSwerveDriveConfig;

public class NavXSwerveDrive extends RioSwerveDrive{

    public final NavXSwerveDriveConfig config;
    private final AHRS navx;

    public NavXSwerveDrive(NavXSwerveDriveConfig config, SwerveModuleBase... modules) {
        super(config, modules);
        this.config = config;

        navx = new AHRS(NavXComType.kMXP_SPI);
        navx.reset();
    }

    @Override
    public Rotation2d getRotation() {
        return navx.getRotation2d();
    }
}
