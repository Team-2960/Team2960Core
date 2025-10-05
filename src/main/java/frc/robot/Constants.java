package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib2960.config.basic.RobotCommonConfig;
import frc.lib2960.config.controller.AngularControllerConfig;
import frc.lib2960.config.controller.FFConfig;
import frc.lib2960.config.controller.LinearControllerConfig;
import frc.lib2960.config.controller.PIDConfig;
import frc.lib2960.config.device.MotorConfig;
import frc.lib2960.helper.LimitTrim;
import frc.lib2960.subsystem.drivetrain.swerve.NavXSwerveDriveConfig;
import frc.lib2960.subsystem.drivetrain.swerve.SwerveDriveCommonConfig;
import frc.lib2960.subsystem.drivetrain.swerve.SwerveModuleBaseConfig;
import frc.lib2960.subsystem.drivetrain.swerve.SwerveModuleCommonConfig;
import frc.lib2960.subsystem.motor.AngularMotorMechConfig;
import frc.lib2960.subsystem.motor.AngularSparkMechConfig;
import frc.lib2960.subsystem.motor.LinearMotorMechConfig;
import frc.lib2960.subsystem.motor.LinearSparkMechConfig;
import frc.lib2960.subsystem.motor.SparkMotorMechConfig.PosEncoderSource;
import frc.lib2960.subsystem.motor.SparkMotorMechConfig.VelEncoderSource;
import frc.lib2960.subsystem.vision.AprilTagPipelineConfig;
import edu.wpi.first.math.Vector;

/**
 * Defines all the robot constants
 */
public class Constants {
    public static final RobotCommonConfig commonConfig = new RobotCommonConfig();

    /*******************/
    /* Driver Controls */
    /*******************/
    public static final int driverCtrlID = 0;
    public static final int opCtrlID = 1;

    public static final double linDriveDB = 0.05;
    public static final double angDriveDB = 0.05;

    /*******************/
    /* Auton Constants */
    /*******************/
    public static final Distance autoClearance = Inches.of(3);
    public static final Distance autoAlignLinTol = Meters.of(.05);
    public static final Angle autoAlignAngTol = Degrees.of(3);

    /*******************/
    /* Robot Constants */
    /*******************/
    public static final Distance coralXOffset = Meters.of(.474);
    public static final Distance coralYOffset = Meters.of(.275);
    public static final Angle coralROffset = Degrees.of(0);

    /**************/
    /* Device IDs */
    /**************/
    public static final int lfDriveMotorID = 9;
    public static final int lfAngleMotorID = 10;
    public static final int lfAngleEncoderID = lfAngleMotorID;

    public static final int rfDriveMotorID = 7;
    public static final int rfAngleMotorID = 8;
    public static final int rfAngleEncoderID = rfAngleMotorID;

    public static final int lrDriveMotorID = 1;
    public static final int lrAngleMotorID = 2;
    public static final int lrAngleEncoderID = lrAngleMotorID;

    public static final int rrDriveMotorID = 5;
    public static final int rrAngleMotorID = 6;
    public static final int rrAngleEncoderID = rrAngleMotorID;

    public static final int coralArmMotorID = 12;
    public static final int coralRollerMotorID = 13;

    public static final int elevatorMotorID = 11;

    public static final int algaeArmMotorID = 3;
    public static final int algaeRollerMotorID = 4;

    public static final int climberMotorID = 14;

    public static final int coralIntakePEID = 15;

    /**********************/
    /* Drivetrain Configs */
    /**********************/
    public static final Distance wheelDiam = Inches.of(2.9);
    public static final double driveGearRatio = 5.08;
    public static final double angleGearRatio = 46.42;
    public static final Current driveMaxCurrent = Amps.of(80);
    public static final Current angleMaxCurrent = Amps.of(40);

    public static final Distance wheelInset = Meters.of(0.044449999999999996);
    public static final Distance frameWidth = Inches.of(29.5);
    public static final Distance frameLength = Inches.of(29.5);
    public static final Distance frameDiag = Inches
            .of(Math.sqrt(Math.pow(frameWidth.in(Inches), 2) + Math.pow(frameLength.in(Inches), 2)));
    public static final Distance bumperThickness = Inches.of(3.25);
    public static final Distance fullWidth = frameWidth.plus(bumperThickness.times(2));
    public static final Distance fullLength = frameLength.plus(bumperThickness.times(2));
    public static final Distance fullDiag = Inches
            .of(Math.sqrt(Math.pow(fullWidth.in(Inches), 2) + Math.pow(fullLength.in(Inches), 2)));

    public static final Distance moduleXOffset = frameLength.div(2).minus(wheelInset);
    public static final Distance moduleYOffset = frameWidth.div(2).minus(wheelInset);

    public static final LinearVelocity driveLinMaxVel = MetersPerSecond.of(4.5);
    public static final LinearAcceleration driveLinMaxAccel = MetersPerSecondPerSecond.of(11);

    public static final AngularVelocity driveAngMaxVel = DegreesPerSecond.of(540);
    public static final AngularAcceleration driveAnglMaxAccel = DegreesPerSecondPerSecond.of(720);

    public static final LinearControllerConfig swerveLinearConfig = new LinearControllerConfig()
            .setMaxVelocity(driveLinMaxVel)
            .setMaxAccel(driveLinMaxAccel)
            .setMaxDecel(driveLinMaxAccel);

    public static final AngularControllerConfig swerveAngularConfig = new AngularControllerConfig()
            .setMaxVelocity(driveAngMaxVel)
            .setMaxAccel(driveAnglMaxAccel);

    public static final LinearControllerConfig driveMotorCtrlConfig = new LinearControllerConfig()
            .setPIDConfig(new PIDConfig(1, 0, 0))
            .setFFConfig(new FFConfig(.08, 2.5));

    public static final AngularControllerConfig angleMotorCtrlConfig = new AngularControllerConfig()
            .setPIDConfig(new PIDConfig(0.05, 0.0, 0.001))
            .setFFConfig(new FFConfig(0.1, 0.1, 0));

    // TODO Add trapezoidal control parameters

    public static final PIDConfig ppLinearPID = new PIDConfig(7, 0, 0);
    public static final PIDConfig ppAngularPID = new PIDConfig(7, 0, 0);

    public static final SwerveDriveCommonConfig swerveDriveCommonConfig = new SwerveDriveCommonConfig()
            .setLinearControlConfig(swerveLinearConfig)
            .setAngularControlConfig(swerveAngularConfig)
            .setLinearPPPID(ppLinearPID)
            .setAngularPPPID(ppAngularPID);

    public static final NavXSwerveDriveConfig swerveDriveConfig = new NavXSwerveDriveConfig()
            .setCommonConfig(swerveDriveCommonConfig);

    public static final SwerveModuleCommonConfig swerveModuleCommonConfig = new SwerveModuleCommonConfig(wheelDiam,
            driveGearRatio, angleGearRatio)
            .setMaxDriveCurrent(driveMaxCurrent)
            .setMaxAngleCurrent(angleMaxCurrent)
            .setDriveControlConfig(driveMotorCtrlConfig)
            .setAngleControlConfig(angleMotorCtrlConfig);

    public static final SwerveModuleBaseConfig lfConfig = new SwerveModuleBaseConfig(
            "Left Front",
            lfDriveMotorID,
            lfAngleMotorID,
            lfAngleEncoderID,
            moduleXOffset,
            moduleYOffset)
            .setInvertDriveMotor(true)
            .setInvertAngleMotor(true)
            .setUITabName("Left Front Swerve");

    public static final SwerveModuleBaseConfig rfConfig = new SwerveModuleBaseConfig(
            "Right Front",
            rfDriveMotorID,
            rfAngleMotorID,
            rfAngleEncoderID,
            moduleXOffset,
            moduleYOffset.unaryMinus())
            .setInvertDriveMotor(false)
            .setInvertAngleMotor(true)
            .setUITabName("Right Front Swerve");

    public static final SwerveModuleBaseConfig lrConfig = new SwerveModuleBaseConfig(
            "Left Rear",
            lrDriveMotorID,
            lrAngleMotorID,
            lrAngleEncoderID,
            moduleXOffset,
            moduleYOffset.unaryMinus())
            .setInvertDriveMotor(true)
            .setInvertAngleMotor(true)
            .setUITabName("Left Rear Swerve");

    public static final SwerveModuleBaseConfig rrConfig = new SwerveModuleBaseConfig(
            "Right Rear",
            rrDriveMotorID,
            rrAngleMotorID,
            rrAngleEncoderID,
            moduleXOffset,
            moduleYOffset.unaryMinus())
            .setInvertDriveMotor(false)
            .setInvertAngleMotor(true)
            .setUITabName("Right Rear Swerve");

    /*********************/
    /* Coral Arm Configs */
    /*********************/
    public static final Angle coralArmPosTol = Degrees.of(2);

    public static final AngularControllerConfig coralArmCtrlConfig = new AngularControllerConfig()
            .setPIDConfig(new PIDConfig(0.002, 0.0, 0.0))
            .setFFConfig(new FFConfig(0.02, 0.025, 0.029481))
            .setMaxVelocity(DegreesPerSecond.of(90))
            .setMaxAccel(DegreesPerSecondPerSecond.of(810))
            .setMaxDecel(DegreesPerSecondPerSecond.of(810))
            .setLimits(Degrees.of(0), Degrees.of(95));

    public static final AngularMotorMechConfig coralArmCommonConfig = new AngularMotorMechConfig("Coral Arm")
            .setUITabName("Coral")
            .setController(angleMotorCtrlConfig)
            .setLimitTrim(LimitTrim.Velocity)
            .addPreset("Intake", Degrees.of(85.5))
            .addPreset("Travel", Degrees.of(75.0))
            .addPreset("L1", Degrees.of(73.0))
            .addPreset("L2", Degrees.of(60.0))
            .addPreset("L3", Degrees.of(60.0))
            .addPreset("L4", Degrees.of(60.0))
            .addPreset("Remove Algae", Degrees.of(45));
    
    public static final AngularSparkMechConfig coralArmConfig = new AngularSparkMechConfig(coralArmCommonConfig)
        .setPosEncoderSource(PosEncoderSource.ABSOLUTE)
        .setVelEncoderSource(VelEncoderSource.ALT_EXT);

    public static final MotorConfig coralArmMotorConfig = new MotorConfig("Coral Arm Motor", coralArmMotorID)
            .setGearRatio(100);

    /************************/
    /* Coral Roller Configs */
    /************************/
    public static final Voltage coralIntakeVolt = Volts.of(4.5);
    public static final Voltage coralEjectVolt = Volts.of(12);
    public static final Voltage coralReverseVolt = Volts.of(-2);

    public static final LinearControllerConfig coralRollerCtrlConfig = new LinearControllerConfig()
            .setPIDConfig(new PIDConfig(3, 0, 0));

    public static final LinearMotorMechConfig coralRollerMechConfig = new LinearMotorMechConfig("Coral Roller",
            Inches.of(2))
            .setUITabName("Coral")
            .setController(coralRollerCtrlConfig)
            .addPreset("Intake", Volts.of(4.5))
            .addPreset("Eject", Volts.of(12))
            .addPreset("Reverse", Volts.of(-2));
            

    public static final MotorConfig coralRollerMotorConfig = new MotorConfig("Coral Roller Motor",
            coralRollerMotorID)
            .setGearRatio(4)
            .setInverted(true);

    public static LinearSparkMechConfig coralRollerConfig = new LinearSparkMechConfig(coralRollerMechConfig);

    /********************/
    /* Elevator Configs */
    /********************/
    public static final Distance elevatorDistPerRev = Inches.of(1.752 * Math.PI / 12);
    public static final Distance elevatorPosTol = Inches.of(.5);

    public static LinearControllerConfig elevatorCtrlConfig = new LinearControllerConfig()
            .setPIDConfig(new PIDConfig(0, 0, 0))
            .setFFConfig(new FFConfig(0.22643, 0.33045, 4.4162, 0.32248))
            .setMaxVelocity(MetersPerSecond.of(0.0508))
            .setMaxAccel(MetersPerSecondPerSecond.of(0.02032))
            .setMaxDecel(MetersPerSecondPerSecond.of(0.02032))
            .setLimits(Meters.of(0.006), Meters.of(1.4605));

    public static LinearMotorMechConfig elevatorCommonConfig = new LinearMotorMechConfig(
            "Elevator",
            elevatorDistPerRev)
            .setUITabName("Coral")
            .setController(coralRollerCtrlConfig)
            .setLimitTrim(LimitTrim.Velocity)
            .addPreset("Intake", Inches.of(0))
            .addPreset("L1", Inches.of(0))
            .addPreset("L2", Inches.of(14))
            .addPreset("L3", Inches.of(29.5))
            .addPreset("L4", Inches.of(54.4))
            .addPreset("Low Algae", Inches.of(4.7))
            .addPreset("High Algae", Inches.of(20));

    public static LinearSparkMechConfig elevatorConfig = new LinearSparkMechConfig(elevatorCommonConfig);

    public static MotorConfig elevatorMotorConfig = new MotorConfig(elevatorMotorID)
            .setGearRatio(12);

    /*********************/
    /* Algae Arm Configs */
    /*********************/
    public static final AngularControllerConfig algaeArmCtrlConfig = new AngularControllerConfig()
            .setFFConfig(new FFConfig(0.15488, 0.64833, 3.8, 0.48409))
            .setMaxVelocity(DegreesPerSecond.of(180))
            .setMaxAccel(DegreesPerSecondPerSecond.of(2160))
            .setMaxDecel(DegreesPerSecondPerSecond.of(2160))
            .setLimits(Degrees.of(10), Degrees.of(88));

    public static final AngularMotorMechConfig algaeArmCommonConfig = new AngularMotorMechConfig("Algae Arm")
            .setUITabName("Algae")
            .setController(angleMotorCtrlConfig)
            .setLimitTrim(LimitTrim.Velocity)
            .addPreset("Home", Degrees.of(80))
            .addPreset("Extended", Degrees.of(20));

    public static final AngularSparkMechConfig algaeArmConfig = new AngularSparkMechConfig(algaeArmCommonConfig)
            .setPosEncoderSource(PosEncoderSource.ABSOLUTE)
            .setVelEncoderSource(VelEncoderSource.INTERNAL);

    public static final MotorConfig algaeArmMotorConfig = new MotorConfig("Algae Arm Motor", algaeArmMotorID)
            .setGearRatio(45);

    /************************/
    /* Algae Roller Configs */
    /************************/
    public static final Voltage algaeIntakeVolt = Volts.of(3);
    public static final Voltage algaeEjectVolt = Volts.of(-1.7);

    public static final AngularMotorMechConfig algaeRollerMechConfig = new AngularMotorMechConfig(
            "Algae Roller")
            .setUITabName("Algae")
            .addPreset("Intake", Volts.of(3))
            .addPreset("Eject", Volts.of(-1.7));

    public static final MotorConfig algaeRollerMotorConfig = new MotorConfig(
            "Algae Roller Motor",
            algaeRollerMotorID)
            .setGearRatio(4)
            .setInverted(true);

    public static final AngularSparkMechConfig algaeRollerConfig = new AngularSparkMechConfig(algaeRollerMechConfig);

    /*******************/
    /* Climber Configs */
    /*******************/
    public static final Voltage climberExtVolt = Volts.of(-6);
    public static final Voltage climberRetVolt = Volts.of(12);

    public static final Angle climberExtAngle = Rotations.of(.325);
    public static final Angle climberRetAngle = Rotations.of(.127);

    public static final AngularMotorMechConfig climberMechConfig = new AngularMotorMechConfig(
            "Climber")
            .setUITabName("Climber")
            .addPreset("Extend", Rotations.of(.325))
            .addPreset("Extend", Volts.of(-6))
            .addPreset("Retract", Rotations.of(.127))
            .addPreset("Retract", Volts.of(12));

    public static final MotorConfig climberMotorConfig = new MotorConfig(
            "Climber Motor",
            climberMotorID)
            .setGearRatio(75);

    public static final AngularSparkMechConfig climberConfig = new AngularSparkMechConfig(climberMechConfig);

    /******************/
    /* Vision Configs */
    /******************/

    public static final Vector<N3> singleStds = VecBuilder.fill(0.5, 0.5, 16);
    public static final Vector<N3> multiStds = VecBuilder.fill(0.5, 0.5, 1);

    public static final Distance maxTagDist = Meters.of(3);
    public static final double ambiguityThreshold = .2;

    public static final Transform3d leftCameraPose = new Transform3d(
            Inches.of(0.876 + 0.125),
            Inches.of(12.357),
            Inches.of(11.068),
            new Rotation3d(
                    Math.toRadians(0),
                    Math.toRadians(0),
                    Math.toRadians(0)));

    public static final Transform3d frontCameraPose = new Transform3d(
            Inches.of(14),
            Inches.of(0.125),
            Inches.of(8.875),
            new Rotation3d(
                    Math.toRadians(0),
                    Math.toRadians(-10),
                    Math.toRadians(0)));

    public static final Transform3d rightCameraPose = new Transform3d(
            Inches.of(13.1),
            Inches.of(-12.614),
            Inches.of(11.068),
            new Rotation3d(
                    Math.toRadians(0),
                    Math.toRadians(0),
                    Math.toRadians(65)));

    public static final AprilTagPipelineConfig leftCameraConfig = new AprilTagPipelineConfig(
            "Camera03",
            leftCameraPose)
            .setFieldLayout(AprilTagFields.k2025ReefscapeWelded)
            .setPoseStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR)
            .setSingleTagSTD(singleStds)
            .setMultiTagSTD(multiStds)
            .setMaxDist(maxTagDist)
            .setAmbiguityThreshold(ambiguityThreshold);

    public static final AprilTagPipelineConfig frontCameraConfig = new AprilTagPipelineConfig(
            "Camera01",
            frontCameraPose)
            .setFieldLayout(AprilTagFields.k2025ReefscapeWelded)
            .setPoseStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR)
            .setSingleTagSTD(VecBuilder.fill(.5, .5, 16))
            .setMultiTagSTD(multiStds)
            .setMaxDist(maxTagDist)
            .setAmbiguityThreshold(ambiguityThreshold);

    public static final AprilTagPipelineConfig rightCameraConfig = new AprilTagPipelineConfig(
            "Camera02",
            rightCameraPose)
            .setFieldLayout(AprilTagFields.k2025ReefscapeWelded)
            .setPoseStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR)
            .setSingleTagSTD(singleStds)
            .setMultiTagSTD(multiStds)
            .setMaxDist(maxTagDist)
            .setAmbiguityThreshold(ambiguityThreshold);

}
