package frc.lib2960.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib2960.settings.NavXSwerveDriveSettings;

public class NavXSwerveDrive extends RioSwerveDrive{

    public final NavXSwerveDriveSettings settings;
    private final AHRS navx;

    public NavXSwerveDrive(NavXSwerveDriveSettings settings, SwerveModuleBase... modules) {
        super(settings, modules);
        this.settings = settings;

        navx = new AHRS(NavXComType.kMXP_SPI);
        navx.reset();
    }

    @Override
    public Rotation2d getRotation() {
        return navx.getRotation2d();
    }
}
