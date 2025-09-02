package frc.lib2960.subsystem.drivetrain.swerve;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib2960.config.subsystem.CTRESwerveDriveConfig;
import frc.lib2960.config.subsystem.SwerveModuleBaseConfig;
import frc.lib2960.config.subsystem.SwerveModuleCommonConfig;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

public class CTRESwerveDrive extends SwerveDriveBase {

    public final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain;

    public final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric();
    public final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric();

    private final SysIdRoutine linearSysIDRoutime;
    private final SysIdRoutine angleSysIDRoutime;
    private final SysIdRoutine turnSysIDRoutime;

    public CTRESwerveDrive(CANBus canBus, CTRESwerveDriveConfig config, SwerveModuleCommonConfig commonConfig,
            SwerveModuleBaseConfig lfConfig, SwerveModuleBaseConfig rfConfig,
            SwerveModuleBaseConfig lrConfig, SwerveModuleBaseConfig rrConfig) {
        super(config);

        // Create DriveTrain Constants
        SwerveDrivetrainConstants drivetrainConstants = new SwerveDrivetrainConstants()
                .withCANBusName(config.CANBusName)
                .withPigeon2Id(config.imuCANID)
                .withPigeon2Configs(null); // TODO Allow Pigion IMU configuration

        // Initialize drivetrain
        var constCreator = getConstCreator(commonConfig);
        drivetrain = new SwerveDrivetrain<TalonFX, TalonFX, CANcoder>(
                TalonFX::new,
                TalonFX::new,
                CANcoder::new,
                drivetrainConstants,
                getModuleConst(constCreator, lfConfig),
                getModuleConst(constCreator, rfConfig),
                getModuleConst(constCreator, lrConfig),
                getModuleConst(constCreator, rrConfig));

        linearSysIDRoutime = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // Use default ramp rate (1 V/s)
                        Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                        null, // Use default timeout (10 s)
                              // Log state with SignalLogger class
                        state -> SignalLogger.writeString("SysIdTranslation_State",
                                state.toString())),
                new SysIdRoutine.Mechanism(
                        output -> drivetrain
                                .setControl(new SwerveRequest.SysIdSwerveTranslation()
                                        .withVolts(output)),
                        null,
                        this));

        angleSysIDRoutime = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // Use default ramp rate (1 V/s)
                        Volts.of(7), // Use dynamic voltage of 7 V
                        null, // Use default timeout (10 s)
                              // Log state with SignalLogger class
                        state -> SignalLogger.writeString("SysIdSteer_State",
                                state.toString())),
                new SysIdRoutine.Mechanism(
                        volts -> drivetrain.setControl(new SwerveRequest.SysIdSwerveSteerGains()
                                .withVolts(volts)),
                        null,
                        this));

        turnSysIDRoutime = new SysIdRoutine(
                new SysIdRoutine.Config(
                        /*
                         * This is in radians per second², but SysId only supports
                         * "volts per second"
                         */
                        Volts.of(Math.PI / 6).per(Second),
                        /* This is in radians per second, but SysId only supports "volts" */
                        Volts.of(Math.PI),
                        null, // Use default timeout (10 s)
                              // Log state with SignalLogger class
                        state -> SignalLogger.writeString("SysIdRotation_State",
                                state.toString())),
                new SysIdRoutine.Mechanism(
                        output -> {
                            /*
                             * output is actually radians per second, but SysId only
                             * supports "volts"
                             */
                            drivetrain.setControl(
                                    new SwerveRequest.SysIdSwerveRotation()
                                            .withRotationalRate(output
                                                    .in(Volts)));
                            /* also log the requested output for SysId */
                            SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                        },
                        null,
                        this));
    }

    @Override
    public void setChassisSpeeds(ChassisSpeeds speeds, boolean isFieldRelative, Distance xOffset,
            Distance yOffset) {

        SwerveRequest driveRequest;

        if (isFieldRelative) {
            driveRequest = fieldCentricDrive
                    .withVelocityX(speeds.vxMetersPerSecond)
                    .withVelocityY(speeds.vyMetersPerSecond)
                    .withRotationalRate(speeds.omegaRadiansPerSecond)
                    .withCenterOfRotation(new Translation2d(xOffset, yOffset));
        } else {
            driveRequest = robotCentricDrive
                    .withVelocityX(speeds.vxMetersPerSecond)
                    .withVelocityY(speeds.vyMetersPerSecond)
                    .withRotationalRate(speeds.omegaRadiansPerSecond)
                    .withCenterOfRotation(new Translation2d(xOffset, yOffset));
        }

        drivetrain.setControl(driveRequest);
    }

    @Override
    public Pose2d getPoseEst() {
        return drivetrain.getState().Pose;
    }

    @Override
    public ChassisSpeeds getFieldRelativeSpeeds() {
        var state = drivetrain.getState();
        return ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, state.Pose.getRotation());
    }

    @Override
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return drivetrain.getState().Speeds;
    }

    @Override
    public void resetPose(Pose2d pose) {
        drivetrain.resetPose(pose);
    }

    @Override
    public void addVisionMeasurement(Pose2d pose, Time timestamp, Vector<N3> std) {
        drivetrain.addVisionMeasurement(pose, timestamp.in(Seconds), std);
    }

    private static SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getConstCreator(
            SwerveModuleCommonConfig commonConfig) {
        Slot0Configs driveGains = new Slot0Configs()
                .withKP(commonConfig.driveCtrlConfig.pidConfig.kP)
                .withKI(commonConfig.driveCtrlConfig.pidConfig.kP)
                .withKD(commonConfig.driveCtrlConfig.pidConfig.kP)
                .withKS(commonConfig.driveCtrlConfig.ffConfig.kS)
                .withKV(commonConfig.driveCtrlConfig.ffConfig.kV);

        Slot0Configs steerGains = new Slot0Configs()
                .withKP(commonConfig.angleCtrlConfig.pidConfig.kP)
                .withKI(commonConfig.angleCtrlConfig.pidConfig.kP)
                .withKD(commonConfig.angleCtrlConfig.pidConfig.kP)
                .withKS(commonConfig.angleCtrlConfig.ffConfig.kS)
                .withKV(commonConfig.angleCtrlConfig.ffConfig.kV)
                .withKA(commonConfig.angleCtrlConfig.ffConfig.kA)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

        TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
        TalonFXConfiguration angleInitialConfigs = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(commonConfig.maxAngleCurrent)
                                .withStatorCurrentLimitEnable(true));

        CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

        SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(commonConfig.driveRatio)
                .withSteerMotorGearRatio(commonConfig.angleRatio)
                .withCouplingGearRatio(commonConfig.coupleRatio)
                .withWheelRadius(commonConfig.wheelRadius)
                .withDriveMotorGains(driveGains)
                .withSteerMotorGains(steerGains)
                .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                .withSlipCurrent(commonConfig.slipCurrent)
                .withSpeedAt12Volts(commonConfig.driveCtrlConfig.maxVel)
                .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
                .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
                .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                .withDriveMotorInitialConfigs(driveInitialConfigs)
                .withSteerMotorInitialConfigs(angleInitialConfigs)
                .withEncoderInitialConfigs(encoderInitialConfigs);

        return constCreator;
    }

    private static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getModuleConst(
            SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constCreator,
            SwerveModuleBaseConfig config) {

        var moduleConst = constCreator.createModuleConstants(
                config.angleMotorID,
                config.driveMotorID,
                config.angleEncoderID,
                config.encoderOffset,
                config.xPos,
                config.yPos,
                config.invertDriveMotor,
                config.invertAngleMotor,
                config.invertAngleEncoder);

        return moduleConst;
    }

    /**
     * Generate a linear motion SysID Command
     * 
     * @param direction   Direction of the command
     * @param quasistatic true for a quasistatic command, false for dynamic command
     * @return new linear motion SysID Command
     */
    @Override
    public Command getLinearSysIdCmd(SysIdRoutine.Direction direction, boolean quasistatic) {
        return (quasistatic) ? linearSysIDRoutime.quasistatic(direction)
                : linearSysIDRoutime.dynamic(direction);
    }

    /**
     * Generate a module angle motion SysID Command
     * 
     * @param direction   Direction of the command
     * @param quasistatic true for a quasistatic command, false for dynamic command
     * @return new module angle motion SysID Command
     */
    @Override
    public Command getAngleSysIdCmd(SysIdRoutine.Direction direction, boolean quasistatic) {
        return (quasistatic) ? angleSysIDRoutime.quasistatic(direction) : angleSysIDRoutime.dynamic(direction);
    }

    /**
     * Generate a robot turn motion SysID Command
     * 
     * @param direction   Direction of the command
     * @param quasistatic true for a quasistatic command, false for dynamic command
     * @return new robot turn motion SysID Command
     */
    @Override
    public Command getTurnSysIdCmd(SysIdRoutine.Direction direction, boolean quasistatic) {
        return (quasistatic) ? turnSysIDRoutime.quasistatic(direction) : turnSysIDRoutime.dynamic(direction);
    }


    /**
     * Creates a new command sequence that includes all the commands to run System
     * Identification on the linear drive motors
     * 
     * @param nextTrigger Trigger for the sequence to move onto the next operation
     *            in the sequence
     * @return new command sequence
     */
    public Command getLinearSysIdSequence(BooleanSupplier nextTrigger) {
        return Commands.sequence(
                Commands.runOnce(SignalLogger::start),
                super.getLinearSysIdSequence(nextTrigger),
                Commands.runOnce(SignalLogger::stop));
    }

    /**
     * Creates a new command sequence that includes all the commands to run System
     * Identification on the angle motors
     * 
     * @param nextTrigger Trigger for the sequence to move onto the next operation
     *                    in the sequence
     * @return new command sequence
     */
    public Command getAngleSysIdSequence(BooleanSupplier nextTrigger) {
        return Commands.sequence(
                Commands.runOnce(SignalLogger::start),
                super.getAngleSysIdSequence(nextTrigger),
                Commands.runOnce(SignalLogger::stop));
    }


    /**
     * Creates a new command sequence that includes all the commands to run System
     * Identification on the robot turning
     * 
     * @param nextTrigger Trigger for the sequence to move onto the next operation
     *                    in the sequence
     * @return new command sequence
     */
    public Command getTurnSysIdSequence(BooleanSupplier nextTrigger) {
        return Commands.sequence(
                Commands.runOnce(SignalLogger::start),
                super.getTurnSysIdSequence(nextTrigger),
                Commands.runOnce(SignalLogger::stop));
    }
}
