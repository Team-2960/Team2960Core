package frc.lib2960.subsystem.drivetrain.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib2960.config.subsystem.SwerveDriveBaseConfig;
import frc.lib2960.helper.AngleUtil;

/**
 * Defines a swerve drive that is managed on the main robot controller
 */
public abstract class RioSwerveDrive extends SwerveDriveBase {

    private final SwerveDriveBaseConfig config;

    private final SwerveModuleBase[] modules;

    private final SwerveDriveKinematics kinematics;

    private final SwerveDrivePoseEstimator poseEst;

    private final SysIdRoutine sysidLinearRoutine;
    private final SysIdRoutine sysidAngleRoutine;
    private final SysIdRoutine sysidTurnRoutine;
    private final MutVoltage sysidDriveVolt = Volts.mutable(0);
    private final MutDistance sysidDrivePos = Meters.mutable(0);
    private final MutLinearVelocity sysidDriveVel = MetersPerSecond.mutable(0);
    private final MutVoltage sysidAngleVolt = Volts.mutable(0);
    private final MutAngle sysidAnglePos = Degrees.mutable(0);
    private final MutAngularVelocity sysidAngleVel = DegreesPerSecond.mutable(0);
    private final MutVoltage sysidTurnVolt = Volts.mutable(0);
    private final MutAngle sysidTurnPos = Degrees.mutable(0);
    private final MutAngularVelocity sysidTurnVel = DegreesPerSecond.mutable(0);

    /**
     * Constructor
     * 
     * @param config  swerve drive config
     * @param modules list of swerve modules
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

        // Initialize SysID
        sysidLinearRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        Volts.of(4),
                        null,
                        null),
                new SysIdRoutine.Mechanism(
                        (volts) -> this.setModuleLinearVolt(volts, new Rotation2d()),
                        this::sysidDriveMotorsLog,
                        this));

        sysidAngleRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        Volts.of(4),
                        null,
                        null),
                new SysIdRoutine.Mechanism(
                        this::setModuleAngleVolt,
                        this::sysidAngleMotorsLog,
                        this));

        sysidTurnRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                    Volts.of(Math.PI / 6).per(Second),
                    Volts.of(Math.PI),
                    null,
                    null),
                new SysIdRoutine.Mechanism(
                        (volts) -> this.setChassisSpeeds(new ChassisSpeeds(0, 0, volts.in(Volts) * Math.PI / 6)),
                        this::sysidTurnMotorsLog,
                        this));
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

    public void setModuleAngleVolt(Voltage voltage) {
        for (var module : modules)
            module.setAngleVolt(voltage);
    }

    public void setModuleLinearVolt(Voltage voltage, Rotation2d angle) {
        for (var module : modules) {
            module.updateAngle(angle);
            module.setDriveVolt(voltage);
        }
    }

    @Override
    public Pose2d getPoseEst() {
        return poseEst.getEstimatedPosition();
    }

    @Override
    public ChassisSpeeds getRobotRelativeSpeeds() {
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

    public abstract AngularVelocity getAngularVelocity();

    public Command getLinearSysIdCmd(SysIdRoutine.Direction direction, boolean quasistatic) {
        return quasistatic ? sysidLinearRoutine.quasistatic(direction) : sysidLinearRoutine.dynamic(direction);
    }

    public Command getAngleSysIdCmd(SysIdRoutine.Direction direction, boolean quasistatic) {
        return quasistatic ? sysidAngleRoutine.quasistatic(direction) : sysidAngleRoutine.dynamic(direction);
    }

    public Command getTurnSysIdCmd(SysIdRoutine.Direction direction, boolean quasistatic) {
        return quasistatic ? sysidTurnRoutine.quasistatic(direction) : sysidTurnRoutine.dynamic(direction);
    }

    public void sysidDriveMotorsLog(SysIdRoutineLog log) {
        for (var module : modules) {
            module.getDriveVoltage(sysidDriveVolt);
            module.getDrivePosition(sysidDrivePos);
            module.getDriveVelocity(sysidDriveVel);

            log.motor(String.format("%s Drive Motor", module.config.name))
                    .voltage(sysidDriveVolt)
                    .linearPosition(sysidDrivePos)
                    .linearVelocity(sysidDriveVel);
        }
    }

    public void sysidAngleMotorsLog(SysIdRoutineLog log) {
        for (var module : modules) {
            module.getAngleVoltage(sysidAngleVolt);
            module.getAnglePosition(sysidAnglePos);
            module.getAngleVelocity(sysidAngleVel);

            log.motor(String.format("%s Angle Motor", module.config.name))
                    .voltage(sysidAngleVolt)
                    .angularPosition(sysidAnglePos)
                    .angularVelocity(sysidAngleVel);
        }
    }

    public void sysidTurnMotorsLog(SysIdRoutineLog log) {
        sysidTurnVel.mut_replace(this.getAngularVelocity());
        sysidTurnVolt.mut_replace(sysidTurnVel.in(RadiansPerSecond), Volts);
        sysidTurnPos.mut_replace(AngleUtil.toUnits(getRotation()));

        log.motor("Robot Turning")
                .voltage(sysidTurnVolt)
                .angularPosition(sysidTurnPos)
                .angularVelocity(sysidTurnVel);
    }
}
