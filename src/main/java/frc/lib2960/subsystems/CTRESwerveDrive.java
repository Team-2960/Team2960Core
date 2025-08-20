package frc.lib2960.subsystems;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.lib2960.settings.CTRESwerveDriveSettings;
import frc.lib2960.settings.SwerveModuleBaseSettings;
import frc.lib2960.settings.SwerveModuleCommonSettings;

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

    public CTRESwerveDrive(CANBus canBus, CTRESwerveDriveSettings settings, SwerveModuleCommonSettings commonSettings,
            SwerveModuleBaseSettings lfSettings, SwerveModuleBaseSettings rfSettings,
            SwerveModuleBaseSettings lrSettings, SwerveModuleBaseSettings rrSettings) {
        super(settings);

        // Create DriveTrain Constants
        SwerveDrivetrainConstants drivetrainConstants = new SwerveDrivetrainConstants()
                .withCANBusName(settings.CANBusName)
                .withPigeon2Id(settings.imuCANID)
                .withPigeon2Configs(null); // TODO Allow Pigion IMU configuration

        // Initialize drivetrain
        var constCreator = getConstCreator(commonSettings);
        drivetrain = new SwerveDrivetrain<TalonFX, TalonFX, CANcoder>(
                TalonFX::new,
                TalonFX::new,
                CANcoder::new,
                drivetrainConstants,
                getModuleConst(constCreator, lfSettings),
                getModuleConst(constCreator, rfSettings),
                getModuleConst(constCreator, lrSettings),
                getModuleConst(constCreator, rrSettings));
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
            SwerveModuleCommonSettings commonSettings) {
        Slot0Configs driveGains = new Slot0Configs()
                .withKP(commonSettings.driveCtrlSettings.pidSettings.kP)
                .withKI(commonSettings.driveCtrlSettings.pidSettings.kP)
                .withKD(commonSettings.driveCtrlSettings.pidSettings.kP)
                .withKS(commonSettings.driveCtrlSettings.ffSettings.kS)
                .withKV(commonSettings.driveCtrlSettings.ffSettings.kV);

        Slot0Configs steerGains = new Slot0Configs()
                .withKP(commonSettings.angleCtrlSettings.pidSettings.kP)
                .withKI(commonSettings.angleCtrlSettings.pidSettings.kP)
                .withKD(commonSettings.angleCtrlSettings.pidSettings.kP)
                .withKS(commonSettings.angleCtrlSettings.ffSettings.kS)
                .withKV(commonSettings.angleCtrlSettings.ffSettings.kV)
                .withKA(commonSettings.angleCtrlSettings.ffSettings.kA)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

        TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
        TalonFXConfiguration angleInitialConfigs = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(commonSettings.maxAngleCurrent)
                                .withStatorCurrentLimitEnable(true));

        CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

        SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(commonSettings.driveRatio)
                .withSteerMotorGearRatio(commonSettings.angleRatio)
                .withCouplingGearRatio(commonSettings.coupleRatio)
                .withWheelRadius(commonSettings.wheelRadius)
                .withDriveMotorGains(driveGains)
                .withSteerMotorGains(steerGains)
                .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                .withSlipCurrent(commonSettings.slipCurrent)
                .withSpeedAt12Volts(commonSettings.driveCtrlSettings.maxVel)
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
            SwerveModuleBaseSettings settings) {

        var moduleConst = constCreator.createModuleConstants(
                settings.angleMotorID,
                settings.driveMotorID,
                settings.angleEncoderID,
                settings.encoderOffset,
                settings.xPos,
                settings.yPos,
                settings.invertDriveMotor,
                settings.invertAngleMotor,
                settings.invertAngleEncoder);

        return moduleConst;
    }
}
