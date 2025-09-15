package frc.lib2960.subsystem.drivetrain.swerve;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import frc.lib2960.config.subsystem.NavXSwerveDriveConfig;

public class NavXSwerveDrive extends RioSwerveDrive {

    public final NavXSwerveDriveConfig config;
    private final AHRS navx;

    public final MutAngularVelocity mutAngularVel = DegreesPerSecond.mutable(0);

    /**
     * Constructor
     * @param config    Swerve drive configuration
     * @param modules   Swerve module objects
     */
    public NavXSwerveDrive(NavXSwerveDriveConfig config, SwerveModuleBase... modules) {
        super(config.common, modules);
        this.config = config;

        navx = new AHRS(NavXComType.kMXP_SPI);
        navx.reset();
    }

    /**
     * Gets the current rotation from the IMU
     * 
     * @return current rotation from the IMU
     */
    @Override
    public Rotation2d getRotation() {
        return navx.getRotation2d();
    }

    /**
     * Gest the current angular velocity from the IMU
     * 
     * @return current angular velocity from the IMU
     */
    @Override
    public AngularVelocity getAngularVelocity() {
        return mutAngularVel.mut_replace(navx.getRate(), DegreesPerSecond);
    }
}
