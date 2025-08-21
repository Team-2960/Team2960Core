package frc.lib2960.subsystems.drivetrain.swerve;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.lib2960.config.SwerveDriveBaseConfig;

/**
 * Defines a swerve drive that is managed on the main robot controller
 */
public abstract class RioSwerveDrive extends SwerveDriveBase {

    private final SwerveDriveBaseConfig config;

    private final SwerveModuleBase[] modules;

    private final SwerveDriveKinematics kinematics;

    private final SwerveDrivePoseEstimator poseEst;

    /**
     * Constructor
     * 
     * @param config swerve drive config
     * @param modules  list of swerve modules
     */
    public RioSwerveDrive(
            SwerveDriveBaseConfig config,
            SwerveModuleBase... modules) {
        super(config);
        this.config = config;
        this.modules = modules;

        // Initialize Kinematics
        var translations = SwerveModuleBase.getTranslations(modules);
        kinematics = new SwerveDriveKinematics(translations);

        // Initialize Pose Estimator
        poseEst = new SwerveDrivePoseEstimator(
                kinematics,
                getRotation(),
                SwerveModuleBase.getPositions(modules),
                new Pose2d(),
                config.stateStd,
                config.visionStd);
    }
    
    @Override
    public void setChassisSpeeds(ChassisSpeeds speeds, boolean isFieldRelative, Distance xOffset, Distance yOffset) {
        // Get offset for center of rotation
        Translation2d offset = new Translation2d(xOffset, yOffset);

        // Convert speeds to robot relative speeds if they are field relative
        if (isFieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPoseEst().getRotation());
        }

        // Discretize speeds for the update period
        speeds = ChassisSpeeds.discretize(speeds, config.period.in(Seconds));

        // Calculate target module states
        var states = kinematics.toSwerveModuleStates(speeds, offset);

        // Set Module States
        for (int i = 0; i < modules.length; i++)
            modules[i].setState(states[i]);
    }

    @Override
    public Pose2d getPoseEst() {
        return poseEst.getEstimatedPosition();
    }

    @Override
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(SwerveModuleBase.getStates(modules));
    }

    @Override
    public void resetPose(Pose2d pose) {
        poseEst.resetPosition(getRotation(), SwerveModuleBase.getPositions(modules), pose);
    }

    public void addVisionMeasurement(Pose2d pose, Time timestamp) {
        addVisionMeasurement(pose, timestamp, config.visionStd);
    }

    @Override
    public void addVisionMeasurement(Pose2d pose, Time timestamp, Vector<N3> std) {
        poseEst.addVisionMeasurement(pose, timestamp.in(Seconds));
    }

    @Override
    public void periodic() {
        super.periodic();

        // Update pose estimation
        // TODO Increase pose estimation rate
        poseEst.update(getRotation(), SwerveModuleBase.getPositions(modules));
    }

    public abstract Rotation2d getRotation();
}
