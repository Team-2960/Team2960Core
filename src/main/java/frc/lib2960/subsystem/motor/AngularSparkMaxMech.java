package frc.lib2960.subsystem.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import frc.lib2960.config.device.MotorConfig;

public class AngularSparkMaxMech extends AngularMotorMech {

    private final SparkMax motor;

    private Optional<AbsoluteEncoder> posAbsEncoder = Optional.empty();
    private Optional<RelativeEncoder> posRelEncoder = Optional.empty();
    private final RelativeEncoder velEncoder;

    public AngularSparkMaxMech(AngularSparkMechConfig config, MotorConfig motorConfig) {
        super(config.commonConfig);

        // Create motor controller
        motor = new SparkMax(motorConfig.id, MotorType.kBrushless);

        // Configure Motor
        SparkMaxConfig flexConfig = new SparkMaxConfig();

        flexConfig.inverted(motorConfig.invert);

        flexConfig.smartCurrentLimit((int) motorConfig.maxMotorCurrent.in(Amps));

        // TODO Implement controller side closed loop control

        // Configure Encoders
        switch (config.posEncoderSource) {
            case INTERNAL:
                flexConfig.encoder.positionConversionFactor(motorConfig.gearRatio);
                break;
            case ABSOLUTE:
                flexConfig.absoluteEncoder.positionConversionFactor(config.posEncoderGearRatio);
                break;
            case ALT_EXT:
                flexConfig.alternateEncoder.positionConversionFactor(config.posEncoderGearRatio);
                break;
        }

        switch (config.velEncoderSource) {
            case INTERNAL:
                flexConfig.encoder.velocityConversionFactor(motorConfig.gearRatio / 60);
                break;
            case ALT_EXT:
                flexConfig.alternateEncoder.positionConversionFactor(config.posEncoderGearRatio / 60);
                break;
        }

        // Apply motor config
        motor.configure(flexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Get Encoders
        switch (config.posEncoderSource) {
            case INTERNAL:
                posRelEncoder = Optional.of(motor.getEncoder());
                break;
            case ABSOLUTE:
                posAbsEncoder = Optional.of(motor.getAbsoluteEncoder());
                break;
            case ALT_EXT:
                posRelEncoder = Optional.of(motor.getAlternateEncoder());
                break;
        }

        switch (config.velEncoderSource) {
            case ALT_EXT:
                velEncoder = motor.getAlternateEncoder();
                break;
            default:
                velEncoder = motor.getEncoder();
                break;

        }

        // Set default command
        setDefaultCommand(getHoldPosCmd());
    }

    @Override
    public void setMotorVoltage(Voltage volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void getPosition(MutAngle result) {
        if (posAbsEncoder.isPresent()) {
            result.mut_replace(posAbsEncoder.get().getPosition(), Rotations);
        } else if (posRelEncoder.isPresent()) {
            result.mut_replace(posRelEncoder.get().getPosition(), Rotations);
        } else {
            result.mut_replace(Rotations.zero());
        }
    }

    @Override
    public void getVelocity(MutAngularVelocity result) {
        result.mut_replace(velEncoder.getVelocity(), RotationsPerSecond);
    }

    @Override
    public void getVoltage(MutVoltage result) {
        result.mut_replace(motor.getAppliedOutput() * motor.getBusVoltage(), Volts);
    }

}
