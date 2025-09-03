package frc.lib2960.subsystem.motor;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2960.config.subsystem.AngularMotorMechConfig;
import frc.lib2960.config.subsystem.MotorMechCommonConfig.LimitTrim;
import frc.lib2960.controller.AngularController;

/**
 * Manages a angular motor mechanism
 */
public abstract class AngularMotorMech extends SubsystemBase {
    // TODO Implement SysID
    // TODO Implement Telemetry
    // TODO Implement Logging
    public final AngularMotorMechConfig config;

    private final AngularController controller;

    private final MutAngle curPos = Radians.mutable(0);
    private final MutAngularVelocity curVel = RadiansPerSecond.mutable(0);
    private final MutVoltage curVolt = Volts.mutable(0);
    private final MutAngularVelocity targetVel = RadiansPerSecond.mutable(0);
    private final MutVoltage targetVolt = Volts.mutable(0);

    /**
     * Constructor
     * 
     * @param config motor mechanism configuration
     */
    public AngularMotorMech(AngularMotorMechConfig config) {
        this.config = config;

        controller = new AngularController(config.controlConfig);
    }

    /**
     * Updates the motor outputs to move to a target position
     * 
     * @param target target position
     */
    public void gotoPosition(Angle target) {
        getPosition(curPos);
        getVelocity(curVel);

        controller.updateVelocity(curPos, curVel, target, targetVel);
    }

    /**
     * Updates the motor outputs to move to a target velocity. If the current
     * position is at a limit, the velocity is trimmed so the mechanism won't exceed
     * the limit.
     * 
     * @param target target velocity
     */
    public void gotoVelocity(AngularVelocity target) {
        getPosition(curPos);
        getVelocity(curVel);

        gotoVelocity(target, curPos, curVel);
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
    public void gotoVelocity(AngularVelocity target, Angle curPos, AngularVelocity curVel) {
        targetVel.mut_replace(target);

        if (config.common.limitTrim == LimitTrim.Velocity)
            controller.trimVelocity(curPos, targetVel);

        controller.updateVoltage(curPos, curVel, target, curVolt);
        setVoltage(targetVolt, curPos);
    }

    /**
     * Sets the motor voltage. If the current position is at a limit, the voltage is
     * trimmed so the mechanism won't exceed the limit.
     * 
     * @param volts
     */
    public void setVoltage(Voltage volts) {
        getPosition(curPos);
        setVoltage(volts, curPos);
    }

    public void setVoltage(Voltage volts, Angle curPos) {
        targetVolt.mut_replace(volts);

        if (config.common.limitTrim == LimitTrim.Voltage)
            controller.trimVoltage(curPos, targetVolt);

        setMotorVoltage(volts);
    }

    /**
     * Sets the output voltage for all the motors
     * @param volts output voltage for all the motors
     */
    public abstract void setMotorVoltage(Voltage volts);

    /**
     * Gets the current position of the mechanism.
     * @param result    mutable object to store the result
     */
    public abstract void getPosition(MutAngle result);

    /**
     * Gets the current velocity of the mechanism.
     * @param result    mutable object to store the result
     */
    public abstract void getVelocity(MutAngularVelocity result);

    /**
     * Gets the current voltage applied to each motor on the mechanism
     * @param result    mutable object to store the result
     */
    public abstract void getVoltage(MutVoltage result);

    /*********************/
    /* Command Factories */
    /*********************/
    /**
     * Gets a command that moves to a target position
     * @param target    target position
     * @return  new command to move to a target position
     */
    public Command getPositionCmd(Angle target) {
        return this.run(() -> this.gotoPosition(target));
    }

    /**
     * Gets a command that moves to a target position. Command finishes automatically when the current position is within tolerance of the target position.
     * @param target    target position
     * @param tolerance target position tolerance
     * @return  new command to move to a target position
     */
    public Command getPositionCmd(Angle target, Angle tolerance) {
        return Commands.deadline(
                getAtTargetCmd(target, tolerance),
                getPositionCmd(target));
    }

    /**
     * Gets a command that moves to a target velocity
     * @param target    target velocity supplier
     * @return  new command to move to a target velocity
     */
    public Command getVelocity(Supplier<AngularVelocity> target) {
        return this.run(() -> this.gotoVelocity(target.get()));
    }

    /**
     * Gets a command that moves to a target velocity
     * @param target    target velocity
     * @return  new command to move to a target velocity
     */
    public Command getVelocity(AngularVelocity target) {
        return this.run(() -> this.gotoVelocity(target));
    }


    /**
     * Gets a command that moves to a target velocity. Command finishes automatically when the current velocity is within tolerance of the target velocity.
     * @param target    target velocity
     * @param tolerance target velocity tolerance
     * @return  new command to move to a target velocity
     */
    public Command getVelocity(AngularVelocity target, AngularVelocity tolerance) {
        return Commands.deadline(
                getAtTargetCmd(target, tolerance),
                getVelocity(target));
    }

    /**
     * Gets a command that moves to a target voltage
     * @param target    target voltage supplier
     * @return  new command to move to a target voltage
     */
    public Command getVoltage(Supplier<Voltage> target) {
        return this.run(() -> this.setVoltage(target.get()));
    }

    /**
     * Gets a command that moves to a target voltage
     * @param target    target voltage
     * @return  new command to move to a target voltage
     */
    public Command getVoltage(Voltage target) {
        return this.run(() -> this.setVoltage(target));
    }

    /**
     * Gets a command that ends when the mechanism is within tolerance of the target velocity.
     * @param target    target position
     * @param tolerance target position tolerance
     * @return
     */
    public Command getAtTargetCmd(Angle target, Angle tolerance) {
        return Commands.waitUntil(
                () -> {
                    getPosition(curPos);
                    return target.isNear(curPos, tolerance);
                });
    }

    /**
     * Gets a command that ends when the mechanism is within tolerance of the target position.
     * @param target    target velocity
     * @param tolerance target velocity tolerance
     * @return  new command to move to a target velocity
     */
    public Command getAtTargetCmd(AngularVelocity target, AngularVelocity tolerance) {
        return Commands.waitUntil(
                () -> {
                    getVelocity(curVel);
                    return target.isNear(curVel, tolerance);
                });
    }
}
