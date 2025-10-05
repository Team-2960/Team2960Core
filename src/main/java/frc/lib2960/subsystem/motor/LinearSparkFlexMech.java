package frc.lib2960.subsystem.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import frc.lib2960.config.device.MotorConfig;

public class LinearSparkFlexMech extends LinearMotorMech {

    private final SparkFlex motor;

    private Optional<AbsoluteEncoder> posAbsEncoder = Optional.empty();
    private Optional<RelativeEncoder> posRelEncoder = Optional.empty();
    private final RelativeEncoder velEncoder;

    public LinearSparkFlexMech(LinearSparkMechConfig config, MotorConfig motorConfig) {
        super(config.commonConfig);

        // Create motor controller
        motor = new SparkFlex(motorConfig.id, MotorType.kBrushless);

        // Configure Motor
        SparkFlexConfig flexConfig = new SparkFlexConfig();

        flexConfig.inverted(motorConfig.invert);

        flexConfig.smartCurrentLimit((int) motorConfig.maxMotorCurrent.in(Amps));

        // TODO Implement controller side closed loop control

        // Configure Encoders
        switch (config.posEncoderSource) {
            case INTERNAL:
                flexConfig.encoder.positionConversionFactor(config.commonConfig.distPerRev.in(Meters));
                break;
            case ABSOLUTE:
                flexConfig.absoluteEncoder.positionConversionFactor(config.posEncoderDistPerRev.in(Meters));
                break;
            case ALT_EXT:
                flexConfig.externalEncoder.positionConversionFactor(config.posEncoderDistPerRev.in(Meters));
                break;
        }

        switch (config.velEncoderSource) {
            case INTERNAL:
                flexConfig.encoder.velocityConversionFactor(config.commonConfig.distPerRev.in(Meters) / 60);
                break;
            case ALT_EXT:
                flexConfig.externalEncoder.positionConversionFactor(config.posEncoderDistPerRev.in(Meters) / 60);
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
                posRelEncoder = Optional.of(motor.getExternalEncoder());
                break;
        }

        switch (config.velEncoderSource) {
            case ALT_EXT:
                velEncoder = motor.getExternalEncoder();
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
    public void getPosition(MutDistance result) {
        if (posAbsEncoder.isPresent()) {
            result.mut_replace(posAbsEncoder.get().getPosition(), Meters);
        } else if (posRelEncoder.isPresent()) {
            result.mut_replace(posRelEncoder.get().getPosition(), Meters);
        } else {
            result.mut_replace(Meters.zero());
        }
    }

    @Override
    public void getVelocity(MutLinearVelocity result) {
        result.mut_replace(velEncoder.getVelocity(), MetersPerSecond);
    }

    @Override
    public void getVoltage(MutVoltage result) {
        result.mut_replace(motor.getAppliedOutput() * motor.getBusVoltage(), Volts);
    }

}
