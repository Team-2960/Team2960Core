package frc.lib2960.subsystem.motor;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib2960.controller.LinearController;
import frc.lib2960.helper.LimitTrim;

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

    protected final ShuffleboardTab tab;

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
            setName("HoldPosCmd");
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
        tab = Shuffleboard.getTab(config.uiTabName);

        controller.addToUI(config.name + " Controller", tab);
        tab.add(config.name + " Subsystem", this);
        tab.add(config.name + " Status", getStatusSendable());
    }

    /**
     * Creates a sendable with the mechanism status
     * 
     * @return sendable with the mechanism status
     */
    public Sendable getStatusSendable() {
        return new Sendable() {
            DistanceUnit posUnit = config.posUnit;
            TimeUnit timeUnit = config.timeUnit;
            LinearVelocityUnit velUnit = posUnit.per(timeUnit);
            String posUnitStr = "(" + posUnit.symbol() + ")";
            String velUnitStr = "(" + posUnit.symbol() + " per " + timeUnit.symbol() + ")";

            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("Current Position " + posUnitStr, () -> getPosition().in(posUnit), null);
                builder.addDoubleProperty("Current Velocity " + velUnitStr, () -> getVelocity().in(velUnit),
                        null);
                builder.addDoubleProperty("Current Voltage (V)", () -> getVoltage().in(Volts), null);
                builder.addDoubleProperty("Target Position " + posUnitStr, () -> targetPos.in(posUnit), null);
                builder.addDoubleProperty("Target Velocity " + velUnitStr, () -> targetVel.in(velUnit), null);
                builder.addDoubleProperty("Target Voltage (V)", () -> targetVolt.in(Volts), null);
            }

        };
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
     * @param volts  sets the target voltage
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
     * @Position current position of the mechanism.
     */
    public Distance getPosition() {
        MutDistance result = Meters.mutable(0);
        getPosition(result);
        return result;
    }

    /**
     * Gets the current velocity of the mechanism.
     * 
     * @Position current velocity of the mechanism.
     */
    public LinearVelocity getVelocity() {
        MutLinearVelocity result = MetersPerSecond.mutable(0);
        getVelocity(result);
        return result;
    }

    /**
     * Gets the current voltage of the mechanism.
     * 
     * @Position current voltage of the mechanism.
     */
    public Voltage getVoltage() {
        MutVoltage result = Volts.mutable(0);
        getVoltage(result);
        return result;
    }

    /**
     * Gets the current position of the mechanism.
     * 
     * @param result mutable object to store the result
     */
    public abstract void getPosition(MutDistance result);

    /**
     * Resets the current position to a known value
     * 
     * @param value known value
     */
    public abstract void resetPosition(Distance value);

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
        Command cmd = this.run(() -> this.gotoPosition(target));
        cmd.setName(String.format("PosCmd: %.0fm", target.in(Meters)));
        return cmd;
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
        Command cmd = Commands.deadline(
                getAtTargetCmd(target, tolerance),
                getPositionCmd(target));

        cmd.setName(String.format("PosCmd and End: %.0fm", target.in(Meters)));

        return cmd;
    }

    /**
     * Gets a command that moves to a target velocity
     * 
     * @param target target velocity supplier
     * @return new command to move to a target velocity
     */
    public Command getVelocityCmd(Supplier<LinearVelocity> target) {
        Command cmd = this.run(() -> this.gotoVelocity(target.get()));

        cmd.setName("VelCtrlCmd");

        return cmd;
    }

    /**
     * Gets a command that moves to a target velocity
     * 
     * @param target target velocity
     * @return new command to move to a target velocity
     */
    public Command getVelocityCmd(LinearVelocity target) {
        Command cmd = this.run(() -> this.gotoVelocity(target));
        cmd.setName(String.format("VelCmd: %.0fm/s", target.in(MetersPerSecond)));
        return cmd;
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
        Command cmd = Commands.deadline(
                getAtTargetCmd(target, tolerance),
                getVelocityCmd(target));

        cmd.setName(String.format("VelCmd and End: %.0fm/s", target.in(MetersPerSecond)));

        return cmd;
    }

    /**
     * Gets a command that moves to a target voltage
     * 
     * @param target target voltage supplier
     * @return new command to move to a target voltage
     */
    public Command getVoltageCmd(Supplier<Voltage> target) {
        Command cmd = this.run(() -> this.setVoltage(target.get()));
        cmd.setName("VoltCtrlCmd");
        return cmd;
    }

    /**
     * Gets a command that moves to a target voltage
     * 
     * @param target target voltage
     * @return new command to move to a target voltage
     */
    public Command getVoltageCmd(Voltage target) {
        Command cmd = this.run(() -> this.setVoltage(target));
        cmd.setName(String.format("VoltCmd: %.0fV", target.in(Volts)));
        return cmd;
    }

    /**
     * Gets a command that moves to a target voltage
     * 
     * @param target target voltage
     * @return new command to move to a target voltage
     */
    public Command getVoltageCmd(Voltage target, Distance pos) {
        Command cmd;
        cmd = Commands.deadline(
                target.gte(Volts.zero()) ? getIsAboveCmd(pos) : getIsBelowCmd(pos),
                getVoltageCmd(target));

        cmd.setName(String.format("VoltToPosCmd: %.0fV %.2fm", target.in(Volts), pos.in(Meters)));
        return cmd;
    }

    /**
     * Gets a command that moves to a target voltage
     * 
     * @param target target voltage
     * @return new command to move to a target voltage
     */
    public Command getVoltageCmd(Voltage target, LinearVelocity vel) {
        Command cmd;
        cmd = Commands.deadline(
                target.gte(Volts.zero()) ? getIsAboveCmd(vel) : getIsBelowCmd(vel),
                getVoltageCmd(target));

        cmd.setName(String.format("VoltToPosCmd: %.0fV %.2fm/s", target.in(Volts), vel.in(MetersPerSecond)));
        return cmd;
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
        Command cmd = Commands.waitUntil(
                () -> {
                    getPosition(curPos);
                    return target.isNear(curPos, tolerance);
                });
        cmd.setName(String.format("AtTargetCmd Pos: %.2fm", target.in(Meters)));
        return cmd;
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
        Command cmd = Commands.waitUntil(
                () -> {
                    getVelocity(curVel);
                    return target.isNear(curVel, tolerance);
                });

        cmd.setName(String.format("AtTargetCmd Vel: %.2fm/s", target.in(MetersPerSecond)));

        return cmd;
    }

    /**
     * Gets a command that waits until the mechanism is above a target position
     * 
     * @param pos target position
     * @return new command
     */
    public Command getIsAboveCmd(Distance pos) {
        Command cmd = Commands.waitUntil(() -> getPosition().gte(pos));

        cmd.setName("IsAboveCmd Pos: %.2fm");

        return cmd;
    }

    /**
     * Gets a command that waits until the mechanism is below a target position
     * 
     * @param pos target position
     * @return new command
     */
    public Command getIsBelowCmd(Distance pos) {
        Command cmd = Commands.waitUntil(() -> getPosition().lte(pos));

        cmd.setName("IsBelowCmd Pos: %.2fm");

        return cmd;
    }

    /**
     * Gets a command that waits until the mechanism is above a target velocity
     * 
     * @param pos target velocity
     * @return new command
     */
    public Command getIsAboveCmd(LinearVelocity pos) {
        Command cmd = Commands.waitUntil(() -> getVelocity().gte(pos));

        cmd.setName("IsAboveCmd Vel: %.2fm/s");

        return cmd;
    }

    /**
     * Gets a command that waits until the mechanism is below a target velocity
     * 
     * @param pos target velocity
     * @return new command
     */
    public Command getIsBelowCmd(LinearVelocity pos) {
        Command cmd = Commands.waitUntil(() -> getVelocity().lte(pos));

        cmd.setName("IsBelowCmd Vel: %.2fm/s");

        return cmd;
    }

    /**
     * Gets a command that ends when the mechanism is at or below its minimum limit
     * 
     * @return new command
     */
    public Command getAtMinCmd() {
        Command cmd = Commands.waitUntil(() -> !controller.aboveMin(getPosition()));

        cmd.setName("AtMinCmd");

        return cmd;
    }

    /**
     * Gets a command that ends when the mechanism is at or below its minimum limit
     * 
     * @return new command
     */
    public Command getAtMaxCmd() {
        Command cmd = Commands.waitUntil(() -> !controller.belowMax(getPosition()));

        cmd.setName("AtMaxCmd");

        return cmd;
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
            Command cmd = getPositionCmd(config.presetPos.get(name));
            cmd.setName(String.format("PosPresetCmd: %s", name));
            return cmd;
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
            Command cmd = getPositionCmd(config.presetPos.get(name), tolerance);
            cmd.setName(String.format("PosPresetCmd and End: %s", name));
            return cmd;
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
            Command cmd = getVelocityCmd(config.presetVel.get(name));
            cmd.setName(String.format("VelPresetCmd: %s", name));
            return cmd;
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
            Command cmd = getVelocityCmd(config.presetVel.get(name), tolerance);
            cmd.setName(String.format("VelPresetCmd and End: %s", name));
            return cmd;
        } else {
            throw new IllegalArgumentException(String.format("No velocity preset with name \"%s\" found.", name));
        }
    }

    /**
     * Gets a new command for moving to a named preset voltage
     * 
     * @param name name of the preset
     * @return new command for moving to a named preset voltage
     * @exception IllegalArgumentException Thrown if the named voltage preset does
     *                                     not exist.
     */
    public Command getVoltPresetCmd(String name) {
        if (config.presetVolt.containsKey(name)) {
            Command cmd = getVoltageCmd(config.presetVolt.get(name));
            cmd.setName(String.format("VoltPresetCmd: %s", name));
            return cmd;
        } else {
            throw new IllegalArgumentException(String.format("No voltage preset with name \"%s\" found.", name));
        }
    }

    /**
     * Gets a new command for moving to a named preset voltage ending at a preset
     * position
     * 
     * @param voltPreset name of the voltage preset
     * @param posPreset  name of the position preset
     * @return new command for moving to a named preset voltage and ending at a
     *         preset position
     * @exception IllegalArgumentException Thrown if the named voltage preset does
     *                                     not exist.
     */
    public Command getVoltPosPresetCmd(String voltPreset, String posPreset) {
        if (config.presetVolt.containsKey(voltPreset)) {
            if (config.presetPos.containsKey(posPreset)) {
                Command cmd = getVoltageCmd(config.presetVolt.get(voltPreset), config.presetPos.get(posPreset));
                cmd.setName(String.format("VoltPosPresetCmd: V-%s P-%s", voltPreset, posPreset));
                return cmd;
            } else {
                throw new IllegalArgumentException(
                        String.format("No position preset with name \"%s\" found.", posPreset));
            }
        } else {
            throw new IllegalArgumentException(String.format("No voltage preset with name \"%s\" found.", voltPreset));
        }
    }

    /**
     * Gets a new command for moving to a named preset voltage ending at a preset
     * velocity
     * 
     * @param voltPreset name of the voltage preset
     * @param velPreset  name of the velocity preset
     * @return new command for moving to a named preset voltage and ending at a
     *         preset velocity
     * @exception IllegalArgumentException Thrown if the named voltage preset does
     *                                     not exist.
     */
    public Command getVoltVelPresetCmd(String voltPreset, String velPreset) {
        if (config.presetVolt.containsKey(voltPreset)) {
            if (config.presetVel.containsKey(velPreset)) {
                Command cmd = getVoltageCmd(config.presetVolt.get(voltPreset), config.presetVel.get(velPreset));
                cmd.setName(String.format("VoltVelPresetCmd: V-%s P-%s", voltPreset, velPreset));
                return cmd;
            } else {
                throw new IllegalArgumentException(
                        String.format("No velocity preset with name \"%s\" found.", velPreset));
            }
        } else {
            throw new IllegalArgumentException(String.format("No voltage preset with name \"%s\" found.", voltPreset));
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
        Command cmd = Commands.sequence(
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

        cmd.setName("SysID Full Sequence Cmd");

        return cmd;
    }
}
