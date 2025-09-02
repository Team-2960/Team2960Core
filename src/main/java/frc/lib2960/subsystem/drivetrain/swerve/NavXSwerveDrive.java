package frc.lib2960.subsystem.drivetrain.swerve;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import frc.lib2960.config.subsystem.NavXSwerveDriveConfig;

public class NavXSwerveDrive extends RioSwerveDrive{

    public final NavXSwerveDriveConfig config;
    private final AHRS navx;

    public final MutAngularVelocity mutAngularVel = DegreesPerSecond.mutable(0);

    public NavXSwerveDrive(NavXSwerveDriveConfig config, SwerveModuleBase... modules) {
        super(config.baseConfig, modules);
        this.config = config;

        navx = new AHRS(NavXComType.kMXP_SPI);
        navx.reset();
    }

    @Override
    public Rotation2d getRotation() {
        return navx.getRotation2d();
    }

    @Override
    public AngularVelocity getAngularVelocity() {
        return mutAngularVel.mut_replace(navx.getRate(), DegreesPerSecond);
    }
}
