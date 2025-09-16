package frc.lib2960.subsystem.motor;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularAcceleration;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearAcceleration;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib2960.config.device.MotorConfig;
import frc.lib2960.config.subsystem.LinearMotorMechConfig;

public class LinearTalonFXMech extends LinearMotorMech {

    private final TalonFX motor;

    private final Supplier<Angle> posSupplier;
    private final Supplier<AngularVelocity> velSupplier;
    private final Supplier<Voltage> voltSupplier;

    private final PositionVoltage posRequest = new PositionVoltage(0);
    private final VelocityVoltage velRequest = new VelocityVoltage(0);
    private final VoltageOut voltRequest = new VoltageOut(0);

    private final MutAngle angleCalc = Rotations.mutable(0);
    private final MutAngularVelocity angVelCalc = RotationsPerSecond.mutable(0);

    /**
     * Constructor
     * 
     * @param config      Motor Mechanism Configuration
     * @param motorConfig Motor Configruration
     */
    public LinearTalonFXMech(LinearMotorMechConfig config, MotorConfig motorConfig) {
        super(config);

        motor = new TalonFX(motorConfig.id, motorConfig.CANBusName);

        // Create TalonFX Config
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();

        // Set Basic Configs
        talonConfig.MotorOutput.withInverted(
                motorConfig.invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive);

        talonConfig.CurrentLimits.withStatorCurrentLimit(motorConfig.maxMotorCurrent);

        // Set Motor Limits
        if (config.controlConfig.minimum.isPresent()) {
            linToAng(config.controlConfig.minimum.get(), angleCalc);

            talonConfig.SoftwareLimitSwitch
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(angleCalc);
        }

        if (config.controlConfig.maximum.isPresent()) {
            linToAng(config.controlConfig.maximum.get(), angleCalc);

            talonConfig.SoftwareLimitSwitch
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(angleCalc);
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
        MutAngularVelocity maxVel = RotationsPerSecond.mutable(0);
        MutAngularAcceleration maxDecel = RotationsPerSecondPerSecond.mutable(0);

        linToAng(config.controlConfig.maxVel, maxVel);
        linToAng(config.controlConfig.maxDecel, maxDecel);

        talonConfig.MotionMagic
                .withMotionMagicCruiseVelocity(maxVel)
                .withMotionMagicAcceleration(maxDecel);
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
     * Sets the position of the motor encoder
     * 
     * @param position new position of the motor encoder
     */
    public void setPosition(Distance position) {
        linToAng(position, angleCalc);
        motor.setPosition(angleCalc);
    }

    /**
     * Updates the motor outputs to move to a target position
     * 
     * @param target target position
     */
    @Override
    public void gotoPosition(Distance target) {
        linToAng(target, angleCalc);
        motor.setControl(posRequest.withPosition(angleCalc));
    }

    /**
     * Updates the motor outputs to move to a target velocity. If the current
     * position is at a limit, the velocity is trimmed so the mechanism won't exceed
     * the limit.
     * 
     * @param target target velocity
     */
    @Override
    public void gotoVelocity(LinearVelocity target) {
        gotoVelocity(target, Meters.zero(), MetersPerSecond.zero());
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
    public void gotoVelocity(LinearVelocity target, Distance curPos, LinearVelocity curVel) {
        linToAng(target, angVelCalc);
        motor.setControl(velRequest.withVelocity(angVelCalc));
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
    public void setVoltage(Voltage volts, Distance curPos) {
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
    public void getPosition(MutDistance result) {
        angToLin(posSupplier.get(), result);
    }

    /**
     * Gets the current velocity of the mechanism.
     * 
     * @param result mutable object to store the result
     */
    @Override
    public void getVelocity(MutLinearVelocity result) {
        angToLin(velSupplier.get(), result);
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

    /**
     * Converts a linear distance to an equivalent angular distance on the motor
     * encoder
     * 
     * @param linear linear distance
     * @param result angular distance
     */
    public void linToAng(Distance linear, MutAngle result) {
        result.mut_replace(
                linear.in(Meters) / config.pulleyCircumference.in(Meters), Rotations);
    }

    /**
     * Converts a linear velocity to an equivalent angular velocity on the motor
     * encoder
     * 
     * @param linear linear velocity
     * @param result angular velocity
     */
    public void linToAng(LinearVelocity linear, MutAngularVelocity result) {
        result.mut_replace(
                linear.in(MetersPerSecond) / config.pulleyCircumference.in(Meters),
                RotationsPerSecond);
    }

    /**
     * Converts a linear acceleration to an equivalent angular acceleration on the
     * motor encoder
     * 
     * @param linear linear acceleration
     * @param result angular acceleration
     */
    public void linToAng(LinearAcceleration linear, MutAngularAcceleration result) {
        result.mut_replace(
                linear.in(MetersPerSecondPerSecond) / config.pulleyCircumference.in(Meters),
                RotationsPerSecondPerSecond);
    }

    /**
     * Converts a angular distance on the motor to an equivalent linear distance
     * 
     * @param angular angular distance
     * @param result  linear distance
     */
    public void angToLin(Angle angular, MutDistance result) {
        result.mut_replace(
                angular.in(Rotations) * config.pulleyCircumference.in(Meters), Meters);
    }

    /**
     * Converts a angular velocity on the motor to an equivalent linear velocity
     * 
     * @param angular angular velocity
     * @param result  linear velocity
     */
    public void angToLin(AngularVelocity angular, MutLinearVelocity result) {
        result.mut_replace(
                angular.in(RotationsPerSecond) * config.pulleyCircumference.in(Meters), MetersPerSecond);
    }

    /**
     * Converts a angular acceleration on the motor to an equivalent linear
     * acceleration
     * 
     * @param angular angular acceleration
     * @param result  linear acceleration
     */
    public void angToLin(AngularAcceleration angular, MutLinearAcceleration result) {
        result.mut_replace(
                angular.in(RotationsPerSecondPerSecond) * config.pulleyCircumference.in(Meters),
                MetersPerSecondPerSecond);
    }

    /*********************/
    /* Command Factories */
    /*********************/
    public Command setPositionCmd(Distance position) {
        return Commands.runOnce(() -> setPosition(position));
    }
}
