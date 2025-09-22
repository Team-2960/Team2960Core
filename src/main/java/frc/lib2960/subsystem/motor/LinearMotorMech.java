package frc.lib2960.subsystem.motor;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib2960.config.subsystem.LinearMotorMechConfig;
import frc.lib2960.controller.LinearController;
import frc.lib2960.helper.LimitTrim;
import frc.lib2960.telemetry.SendableMeasure;

/**
 * Manages a linear motor mechanism
 */
public abstract class LinearMotorMech extends SubsystemBase {
    public final LinearMotorMechConfig config;

    private final LinearController controller;

    private final MutDistance curPos = Meters.mutable(0);
    private final MutLinearVelocity curVel = MetersPerSecond.mutable(0);
    private final MutVoltage curVolt = Volts.mutable(0);
    private final MutDistance targetPos = Meters.mutable(0);
    private final MutLinearVelocity targetVel = MetersPerSecond.mutable(0);
    private final MutVoltage targetVolt = Volts.mutable(0);

    private final SysIdRoutine sysIdRoutine;
    private final MutVoltage sysIdVolt = Volts.mutable(0);
    private final MutDistance sysIdPos = Meters.mutable(0);
    private final MutLinearVelocity sysIdVel = MetersPerSecond.mutable(0);

    private final ShuffleboardLayout layout;

    /**
     * Command to hold position the mechanism is at when the command is scheduled
     */
    private class HoldPosCmd extends Command {
        /** < Record of the current position of the mechanism */
        private MutDistance curPos = Meters.mutable(0);

        /**
         * Constructor
         */
        public HoldPosCmd() {
            addRequirements(LinearMotorMech.this);
        }

        /**
         * Captures the mechanism position when the command is scheduled
         */
        @Override
        public void initialize() {
            getPosition(curPos);
        }

        /**
         * Moves to the captured position
         */
        @Override
        public void execute() {
            gotoPosition(curPos);
        }
    }

    /**
     * Constructor
     * 
     * @param config motor mechanism configuration
     */
    public LinearMotorMech(LinearMotorMechConfig config) {
        this.config = config;

        this.setName(config.name);

        controller = new LinearController(config.controlConfig);

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        this::setVoltage,
                        this::sysIDLog,
                        this));

        // Configure telemetry
        layout =  Shuffleboard.getTab(config.uiTabName)
            .getLayout(config.name, BuiltInLayouts.kList)
            .withSize(2,6); // TODO Optimize
        
        layout.add("Controller", controller);
        layout.add("Subsystem", this);
        layout.add("Current Position", new SendableMeasure<>(curPos));
        layout.add("Current Velocity", new SendableMeasure<>(curVel));
        layout.add("Current Voltage", new SendableMeasure<>(curVolt));
        layout.add("Target Position", new SendableMeasure<>(targetPos));
        layout.add("Target Velocity", new SendableMeasure<>(targetVel));
        layout.add("Target Voltage", new SendableMeasure<>(targetVolt));
    }

    /**
     * Updates the motor outputs to move to a target position
     * 
     * @param target target position
     */
    public void gotoPosition(Distance target) {
        getPosition(curPos);
        getVelocity(curVel);
        targetPos.mut_replace(target);

        controller.updateVelocity(curPos, curVel, target, targetVel);
        gotoVelocity(targetVel, curPos, curVel);
    }

    /**
     * Updates the motor outputs to move to a target velocity. If the current
     * position is at a limit, the velocity is trimmed so the mechanism won't exceed
     * the limit.
     * 
     * @param target target velocity
     */
    public void gotoVelocity(LinearVelocity target) {
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
    public void gotoVelocity(LinearVelocity target, Distance curPos, LinearVelocity curVel) {
        targetVel.mut_replace(target);

        if (config.limitTrim == LimitTrim.Velocity)
            controller.trimVelocity(curPos, targetVel);

        controller.updateVoltage(curVel, target, curVolt);
        setVoltage(targetVolt, curPos);
    }

    /**
     * Sets the motor voltage. If the current position is at a limit, the voltage is
     * trimmed so the mechanism won't exceed the limit.
     * 
     * @param volts sets the target voltage
     */
    public void setVoltage(Voltage volts) {
        getPosition(curPos);
        setVoltage(volts, curPos);
    }

    /**
     * Sets the motor voltage. If the current position is at a limit, the voltage is
     * trimmed so the mechanism won't exceed the limit.
     * 
     * @param volts sets the target voltage
     * @param curPos current mechanism position
     */
    public void setVoltage(Voltage volts, Distance curPos) {
        targetVolt.mut_replace(volts);

        if (config.limitTrim == LimitTrim.Voltage)
            controller.trimVoltage(curPos, targetVolt);

        setMotorVoltage(volts);
    }

    /**
     * Sets the output voltage for all the motors
     * 
     * @param volts output voltage for all the motors
     */
    public abstract void setMotorVoltage(Voltage volts);

    /**
     * Gets the current position of the mechanism.
     * 
     * @param result mutable object to store the result
     */
    public abstract void getPosition(MutDistance result);

    /**
     * Gets the current velocity of the mechanism.
     * 
     * @param result mutable object to store the result
     */
    public abstract void getVelocity(MutLinearVelocity result);

    /**
     * Gets the current voltage applied to each motor on the mechanism
     * 
     * @param result mutable object to store the result
     */
    public abstract void getVoltage(MutVoltage result);

    /*********************/
    /* Command Factories */
    /*********************/
    /**
     * Gets a command that moves to a target position
     * 
     * @param target target position
     * @return new command to move to a target position
     */
    public Command getPositionCmd(Distance target) {
        return this.run(() -> this.gotoPosition(target));
    }

    /**
     * Gets a command that moves to a target position. Command finishes
     * automatically when the current position is within tolerance of the target
     * position.
     * 
     * @param target    target position
     * @param tolerance target position tolerance
     * @return new command to move to a target position
     */
    public Command getPositionCmd(Distance target, Distance tolerance) {
        return Commands.deadline(
                getAtTargetCmd(target, tolerance),
                getPositionCmd(target));
    }

    /**
     * Gets a command that moves to a target velocity
     * 
     * @param target target velocity supplier
     * @return new command to move to a target velocity
     */
    public Command getVelocityCmd(Supplier<LinearVelocity> target) {
        return this.run(() -> this.gotoVelocity(target.get()));
    }

    /**
     * Gets a command that moves to a target velocity
     * 
     * @param target target velocity
     * @return new command to move to a target velocity
     */
    public Command getVelocityCmd(LinearVelocity target) {
        return this.run(() -> this.gotoVelocity(target));
    }

    /**
     * Gets a command that moves to a target velocity. Command finishes
     * automatically when the current velocity is within tolerance of the target
     * velocity.
     * 
     * @param target    target velocity
     * @param tolerance target velocity tolerance
     * @return new command to move to a target velocity
     */
    public Command getVelocityCmd(LinearVelocity target, LinearVelocity tolerance) {
        return Commands.deadline(
                getAtTargetCmd(target, tolerance),
                getVelocityCmd(target));
    }

    /**
     * Gets a command that moves to a target voltage
     * 
     * @param target target voltage supplier
     * @return new command to move to a target voltage
     */
    public Command getVoltageCmd(Supplier<Voltage> target) {
        return this.run(() -> this.setVoltage(target.get()));
    }

    /**
     * Gets a command that moves to a target voltage
     * 
     * @param target target voltage
     * @return new command to move to a target voltage
     */
    public Command getVoltageCmd(Voltage target) {
        return this.run(() -> this.setVoltage(target));
    }

    /**
     * Gets a new hold position command
     * 
     * @return new hold position command
     */
    public Command getHoldPosCmd() {
        return new HoldPosCmd();
    }

    /**
     * Gets a command that ends when the mechanism is within tolerance of the target
     * velocity.
     * 
     * @param target    target position
     * @param tolerance target position tolerance
     * @return new command
     */
    public Command getAtTargetCmd(Distance target, Distance tolerance) {
        return Commands.waitUntil(
                () -> {
                    getPosition(curPos);
                    return target.isNear(curPos, tolerance);
                });
    }

    /**
     * Gets a command that ends when the mechanism is within tolerance of the target
     * position.
     * 
     * @param target    target velocity
     * @param tolerance target velocity tolerance
     * @return new command
     */
    public Command getAtTargetCmd(LinearVelocity target, LinearVelocity tolerance) {
        return Commands.waitUntil(
                () -> {
                    getVelocity(curVel);
                    return target.isNear(curVel, tolerance);
                });
    }

    /**
     * Gets a command that ends when the mechanism is at or below its minimum limit
     * 
     * @return new command
     */
    public Command getAtMinCmd() {
        return Commands.waitUntil(() -> {
            MutDistance dist = Meters.mutable(0);
            getPosition(dist);
            return !controller.aboveMin(dist);
        });
    }

    /**
     * Gets a command that ends when the mechanism is at or below its minimum limit
     * 
     * @return new command
     */
    public Command getAtMaxCmd() {
        return Commands.waitUntil(() -> {
            MutDistance dist = Meters.mutable(0);
            getPosition(dist);
            return !controller.belowMax(dist);
        });
    }

    /**
     * Gets a new command for moving to a named preset position
     * 
     * @param name name of the preset
     * @return new command for moving to a named preset position
     * @exception IllegalArgumentException Thrown if the named position preset does
     *                                     not exist.
     */
    public Command getPosPresetCmd(String name) {
        if (config.presetPos.containsKey(name)) {
            return getPositionCmd(config.presetPos.get(name));
        } else {
            throw new IllegalArgumentException(String.format("No position preset with name \"%s\" found.", name));
        }
    }

    /**
     * Gets a new command for moving to a named preset position. Command ends
     * execution when it is within tolerance of the preset.
     * 
     * @param name      name of the preset
     * @param tolerance
     * @return new command for moving to a named preset position
     * @exception IllegalArgumentException Thrown if the named position preset does
     *                                     not exist.
     */
    public Command getPosPresetCmd(String name, Distance tolerance) {
        if (config.presetPos.containsKey(name)) {
            return getPositionCmd(config.presetPos.get(name), tolerance);
        } else {
            throw new IllegalArgumentException(String.format("No position preset with name \"%s\" found.", name));
        }
    }

    /**
     * Gets a new command for moving to a named preset velocity
     * 
     * @param name name of the preset
     * @return new command for moving to a named preset velocity
     * @exception IllegalArgumentException Thrown if the named velocity preset does
     *                                     not exist.
     */
    public Command getVelPresetCmd(String name) {
        if (config.presetVel.containsKey(name)) {
            return getVelocityCmd(config.presetVel.get(name));
        } else {
            throw new IllegalArgumentException(String.format("No velocity preset with name \"%s\" found.", name));
        }
    }

    /**
     * Gets a new command for moving to a named preset velocity. Command ends
     * execution when it is within tolerance of the preset.
     * 
     * @param name      name of the preset
     * @param tolerance
     * @return new command for moving to a named preset velocity
     * @exception IllegalArgumentException Thrown if the named velocity preset does
     *                                     not exist.
     */
    public Command getVelPresetCmd(String name, LinearVelocity tolerance) {
        if (config.presetVel.containsKey(name)) {
            return getVelocityCmd(config.presetVel.get(name), tolerance);
        } else {
            throw new IllegalArgumentException(String.format("No velocity preset with name \"%s\" found.", name));
        }
    }

    /***************************/
    /* SysID Command Factories */
    /***************************/

    /**
     * Logs sysId data
     * 
     * @param log sysId routime log object
     */
    public void sysIDLog(SysIdRoutineLog log) {
        getVoltage(sysIdVolt);
        getPosition(sysIdPos);
        getVelocity(sysIdVel);

        log.motor(config.name)
                .voltage(sysIdVolt)
                .linearPosition(sysIdPos)
                .linearVelocity(sysIdVel);
    }

    /**
     * Creates a sysID Command
     * 
     * @param direction   direction of the command
     * @param quasistatic true to get a quasistatic command, false to get a dynamic
     *                    command
     * @return new sysID Command
     */
    public Command getSysIdCmd(SysIdRoutine.Direction direction, boolean quasistatic) {
        return quasistatic ? sysIdRoutine.quasistatic(direction) : sysIdRoutine.dynamic(direction);
    }

    /**
     * Generates a full sysID command sequence
     * 
     * @param nextTrigger boolean supplier to move to the next step in the sequence
     * @return new full sysID command sequence
     */
    public Command getTurnSysIdSequence(BooleanSupplier nextTrigger) {
        return Commands.sequence(
                Commands.deadline(
                        Commands.race(
                                Commands.waitUntil(nextTrigger),
                                getAtMaxCmd()),
                        getSysIdCmd(SysIdRoutine.Direction.kForward, true)),
                Commands.waitUntil(() -> !nextTrigger.getAsBoolean()),
                Commands.deadline(
                        Commands.race(
                                Commands.waitUntil(nextTrigger),
                                getAtMinCmd()),
                        getSysIdCmd(SysIdRoutine.Direction.kReverse, true)),
                Commands.waitUntil(() -> !nextTrigger.getAsBoolean()),
                Commands.deadline(
                        Commands.race(
                                Commands.waitUntil(nextTrigger),
                                getAtMaxCmd()),
                        getSysIdCmd(SysIdRoutine.Direction.kForward, false)),
                Commands.waitUntil(() -> !nextTrigger.getAsBoolean()),
                Commands.deadline(
                        Commands.race(
                                Commands.waitUntil(nextTrigger),
                                getAtMinCmd()),
                        getSysIdCmd(SysIdRoutine.Direction.kReverse, false)));
    }
}
