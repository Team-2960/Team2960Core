package frc.lib2960.subsystems;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.lib2960.config.CTRESwerveDriveConfig;
import frc.lib2960.config.SwerveModuleBaseConfig;
import frc.lib2960.config.SwerveModuleCommonConfig;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

public class CTRESwerveDrive extends SwerveDriveBase {

    public final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain;

    public final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric();
    public final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric();

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
    }

    @Override
    public void setChassisSpeeds(ChassisSpeeds speeds, boolean isFieldRelative, Distance xOffset, Distance yOffset) {

        SwerveRequest driveRequest;

        if(isFieldRelative) {
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
    public ChassisSpeeds getChassisSpeeds() {
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
}
