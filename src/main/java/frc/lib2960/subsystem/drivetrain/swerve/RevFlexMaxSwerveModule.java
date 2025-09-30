package frc.lib2960.subsystem.drivetrain.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

public class RevFlexMaxSwerveModule extends SwerveModuleBase {
    private final SparkFlex driveMotor;
    private final SparkMax angleMotor;

    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder angleAbsEncoder;
    private final RelativeEncoder angleVelEncoder;

    /**
     * Constructor
     * 
     * @param commonConfig Swerve module common config
     * @param config       Swerve module config
     */
    public RevFlexMaxSwerveModule(SwerveModuleCommonConfig commonConfig, SwerveModuleBaseConfig config) {
        super(commonConfig, config);

        // Create motors
        driveMotor = new SparkFlex(config.driveMotorID, MotorType.kBrushless);
        angleMotor = new SparkMax(config.angleMotorID, MotorType.kBrushless);

        // Configure Drive Motor
        var driveConfig = new SparkFlexConfig();

        double driveDist = commonConfig.driveRatio * commonConfig.wheelCircumference.in(Meters);

        driveConfig.inverted(config.invertDriveMotor);
        driveConfig.encoder.positionConversionFactor(driveDist);
        driveConfig.encoder.velocityConversionFactor(driveDist / 60);

        driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // Configure Angle Motor
        var angleConfig = new SparkMaxConfig();

        angleConfig.inverted(config.invertAngleMotor);
        angleConfig.absoluteEncoder.inverted(config.invertAngleEncoder)
                .zeroOffset(config.angleEncoderOffset.in(Rotations));
       
        angleConfig.encoder.velocityConversionFactor(commonConfig.angleRatio / 60.0);

       // Can't configure both an alternate encoder and absolute encoder on a spark max

        angleMotor.configure(angleConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        driveEncoder = driveMotor.getEncoder();

        angleAbsEncoder = angleMotor.getAbsoluteEncoder();
        angleVelEncoder = angleMotor.getEncoder();
    }

    /**
     * Sets the voltage of the drive motor
     * 
     * @param volts voltage to set to the drive motor
     */
    @Override
    public void setDriveVolt(Voltage volts) {
        driveMotor.setVoltage(volts.in(Volts));
    }

    /**
     * Sets the voltage of the angle motor
     * 
     * @param volts voltage to set to the angle motor
     */
    @Override
    public void setAngleVolt(Voltage volts) {
        angleMotor.setVoltage(volts.in(Volts));
    }

    /**
     * Gets the current drive position
     * 
     * @param result mutable object to store the result
     */
    @Override
    public void getDrivePos(MutDistance result) {
        result.mut_replace(driveEncoder.getPosition(), Meters);
    }

    /**
     * Gets the current drive velocity
     * 
     * @param result mutable object to store the result
     */
    @Override
    public void getDriveVel(MutLinearVelocity result) {
        result.mut_replace(driveEncoder.getVelocity(), MetersPerSecond);
    }

    /**
     * Gets the current drive motor applied voltage
     * 
     * @param result
     */
    @Override
    public void getDriveVolt(MutVoltage result) {
        result.mut_replace(driveMotor.getAppliedOutput() * driveMotor.getBusVoltage(), Volts);
    }

    /**
     * Gets the current angle position
     * 
     * @param result mutable object to store the result
     */
    @Override
    public void getAnglePos(MutAngle result) {
        result.mut_replace(angleAbsEncoder.getPosition(), Rotations);
    }

    /**
     * Gets the current angle velocity
     * 
     * @param result mutable object to store the result
     */
    @Override
    public void getAngleVel(MutAngularVelocity result) {
        result.mut_replace(angleVelEncoder.getVelocity(), RotationsPerSecond);
    }

    /**
     * Gets the current angle motor applied voltage
     * 
     * @param result mutable object to store the result
     */
    @Override
    public void getAngleVolt(MutVoltage result) {
        result.mut_replace(angleMotor.getAppliedOutput() * angleMotor.getBusVoltage(), Volts);
    }

}
