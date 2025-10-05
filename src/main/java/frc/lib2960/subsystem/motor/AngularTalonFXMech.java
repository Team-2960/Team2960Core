package frc.lib2960.subsystem.motor;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib2960.config.device.EncoderConfig;
import frc.lib2960.config.device.MotorConfig;

public class AngularTalonFXMech extends AngularMotorMech {

    private final TalonFX motor;
    @SuppressWarnings("unused")
    private Optional<CANcoder> encoder;

    private final Supplier<Angle> posSupplier;
    private final Supplier<AngularVelocity> velSupplier;
    private final Supplier<Voltage> voltSupplier;

    private final PositionVoltage posRequest = new PositionVoltage(0);
    private final VelocityVoltage velRequest = new VelocityVoltage(0);
    private final VoltageOut voltRequest = new VoltageOut(0);

    /**
     * Constructor
     * 
     * @param config      Motor Mechanism Configuration
     * @param motorConfig Motor Configruration
     */
    public AngularTalonFXMech(AngularMotorMechConfig config, MotorConfig motorConfig) {
        super(config);

        motor = new TalonFX(motorConfig.id, motorConfig.CANBusName);
        encoder = Optional.empty();

        // Create TalonFX Config
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();

        // Set Basic Configs
        talonConfig.MotorOutput.withInverted(
                motorConfig.invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive);

        talonConfig.CurrentLimits.withStatorCurrentLimit(motorConfig.maxMotorCurrent);

        // Set Motor Limits
        if (config.controlConfig.minimum.isPresent()) {
            talonConfig.SoftwareLimitSwitch
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(config.controlConfig.minimum.get());
        }

        if (config.controlConfig.maximum.isPresent()) {
            talonConfig.SoftwareLimitSwitch
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(config.controlConfig.maximum.get());
        }

        // Set Controller loops
        talonConfig.Slot0
                .withGravityType(GravityTypeValue.Elevator_Static)
                .withKP(config.controlConfig.pidConfig.kP)
                .withKI(config.controlConfig.pidConfig.kI)
                .withKD(config.controlConfig.pidConfig.kD)
                .withKS(config.controlConfig.ffConfig.kS)
                .withKV(config.controlConfig.ffConfig.kV)
                .withKG(config.controlConfig.ffConfig.kG)
                .withKA(config.controlConfig.ffConfig.kA);

        // Set Motion Profiling Settings
        talonConfig.MotionMagic
                .withMotionMagicCruiseVelocity(config.controlConfig.maxVel)
                .withMotionMagicAcceleration(config.controlConfig.maxDecel);
        // TODO Set MotionMagic kA and kV

        // Configure encoder
        talonConfig.Feedback.withRotorToSensorRatio(motorConfig.gearRatio);

        // Set Motor Config
        motor.getConfigurator().apply(talonConfig);

        // Get motor sensor suppliers
        posSupplier = motor.getPosition().asSupplier();
        velSupplier = motor.getVelocity().asSupplier();
        voltSupplier = motor.getMotorVoltage().asSupplier();
    }

    /**
     * Constructor
     * 
     * @param config      Motor Mechanism Configuration
     * @param motorConfig Motor Configruration
     */
    public AngularTalonFXMech(AngularMotorMechConfig config, MotorConfig motorConfig, EncoderConfig encoderConfig,
            FeedbackSensorSourceValue sensorSourceType) {
        this(config, motorConfig);
        // TODO: Allow other encoder types
        encoder = Optional.of(new CANcoder(encoderConfig.id, encoderConfig.CANBusName));

        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.withSensorDirection(encoderConfig.invert ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive);

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();

        feedbackConfigs
                .withRotorToSensorRatio(motorConfig.gearRatio)
                .withFeedbackRemoteSensorID(encoderConfig.id)
                .withFeedbackSensorSource(sensorSourceType)
                .withSensorToMechanismRatio(encoderConfig.gearRatio);

        motor.getConfigurator().apply(feedbackConfigs);
    }

    /**
     * Sets the position of the motor encoder
     * 
     * @param position new position of the motor encoder
     */
    public void setPosition(Angle position) {
        motor.setPosition(position);
    }
    

    /**
     * Resets the current position to a known value
     * 
     * @param value known value
     */
    @Override
    public void resetPosition(Angle value) {
        if(encoder.isPresent()) {
            encoder.get().setPosition(value);
        } else {
            motor.setPosition(value);
        }
    }

    /**
     * Updates the motor outputs to move to a target position
     * 
     * @param target target position
     */
    @Override
    public void gotoPosition(Angle target) {
        motor.setControl(posRequest.withPosition(target));
    }

    /**
     * Updates the motor outputs to move to a target velocity. If the current
     * position is at a limit, the velocity is trimmed so the mechanism won't exceed
     * the limit.
     * 
     * @param target target velocity
     */
    @Override
    public void gotoVelocity(AngularVelocity target) {
        gotoVelocity(target, Rotations.zero(), RotationsPerSecond.zero());
    }

    /**
     * Updates the motor outputs to move to a target velocity. If the current
     * position is at a limit, the velocity is trimmed so the mechanism won't exceed
     * the limit.
     * 
     * @param target target velocity
     * @param curPos current position
     * @param curVel current velocity
     */
    @Override
    public void gotoVelocity(AngularVelocity target, Angle curPos, AngularVelocity curVel) {
        motor.setControl(velRequest.withVelocity(target));
    }

    /**
     * Sets the motor voltage. If the current position is at a limit, the voltage is
     * trimmed so the mechanism won't exceed the limit.
     * 
     * @param volts sets the target voltage
     */
    @Override
    public void setVoltage(Voltage volts) {
        setMotorVoltage(volts);
    }

    /**
     * Sets the motor voltage. If the current position is at a limit, the voltage is
     * trimmed so the mechanism won't exceed the limit.
     * 
     * @param volts  sets the target voltage
     * @param curPos current mechanism position
     */
    @Override
    public void setVoltage(Voltage volts, Angle curPos) {
        setMotorVoltage(volts);
    }

    /**
     * Sets the output voltage for all the motors
     * 
     * @param volts output voltage for all the motors
     */
    @Override
    public void setMotorVoltage(Voltage volts) {
        motor.setControl(voltRequest.withOutput(volts));
    }

    /**
     * Gets the current position of the mechanism.
     * 
     * @param result mutable object to store the result
     */
    @Override
    public void getPosition(MutAngle result) {
        result.mut_replace(posSupplier.get());
    }

    /**
     * Gets the current velocity of the mechanism.
     * 
     * @param result mutable object to store the result
     */
    @Override
    public void getVelocity(MutAngularVelocity result) {
        result.mut_replace(velSupplier.get());
    }

    /**
     * Gets the current voltage applied to each motor on the mechanism
     * 
     * @param result mutable object to store the result
     */
    @Override
    public void getVoltage(MutVoltage result) {
        result.mut_replace(voltSupplier.get());
    }

    /*********************/
    /* Command Factories */
    /*********************/
    public Command setPositionCmd(Angle position) {
        return Commands.runOnce(() -> setPosition(position));
    }
}
