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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2960.config.subsystem.AngularMotorMechConfig;
import frc.lib2960.config.subsystem.MotorMechCommonConfig.LimitTrim;
import frc.lib2960.controller.AngularController;

/**
 * Manages a angular motor mechanism
 */
public abstract class AngularMotorMech extends SubsystemBase {
    // TODO Implement SysID
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

    public abstract void setMotorVoltage(Voltage volts);

    public abstract void getPosition(MutAngle result);

    public abstract void getVelocity(MutAngularVelocity result);

    public abstract void getVoltage(MutVoltage result);


    /*********************/
    /* Command Factories */
    /*********************/
    public Command getPositionCmd(Angle position) {
        return this.run(() -> this.gotoPosition(position));
    }

    public Command getVelocity(Supplier<AngularVelocity> velocity) {
        return this.run(() -> this.gotoVelocity(velocity.get()));
    }

    public Command getVelocity(AngularVelocity velocity) {
        return this.run(() -> this.gotoVelocity(velocity));
    }

    public Command getVoltage(Supplier<Voltage> velocity) {
        return this.run(() -> this.setVoltage(velocity.get()));
    }

    public Command getVoltage(Voltage velocity) {
        return this.run(() -> this.setVoltage(velocity));
    }

}
