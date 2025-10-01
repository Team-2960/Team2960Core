package frc.lib2960.controller;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib2960.config.controller.AngularControllerConfig;
import frc.lib2960.telemetry.SendableArmFeedForward;

public class AngularController {
    public final AngularControllerConfig config;
    private final TrapezoidalProfile trapProfile;
    private final PIDController pid;
    private final ArmFeedforward ff;

    /**
     * Constructor
     * 
     * @param config Controller config
     */
    public AngularController(AngularControllerConfig config) {
        this.config = config;
        trapProfile = new TrapezoidalProfile(
                config.maxVel.in(DegreesPerSecond),
                config.maxAccel.in(DegreesPerSecondPerSecond),
                config.maxDecel.in(DegreesPerSecondPerSecond),
                config.period.in(Seconds));
        pid = config.pidConfig.getPIDController();
        ff = config.ffConfig.getArmFF();
    }

    /**
     * Adds controller objects to a ShuffleboardLayout
     * 
     * @param name name of the controller for shuffleboard
     * @param tab  shuffle board tab to add controller objects to
     */
    public void addToUI(String name, ShuffleboardTab tab) {
        tab.add(name + " PID", pid);
        tab.add(name + " FeedForward", new SendableArmFeedForward(ff));
        tab.add(name + " Profile", trapProfile);
    }

    /**
     * Adds controller objects to a ShuffleboardLayout
     * 
     * @param name   name of the controller for shuffleboard
     * @param layout shuffle board layout to add controller objects to
     */
    public void addToUI(String name, ShuffleboardLayout layout) {
        layout.add(name + " PID", pid);
        layout.add(name + " FeedForward", new SendableArmFeedForward(ff));
        layout.add(name + " Profile", trapProfile);
    }

    /**
     * Calculate the target rate for the trapezoidal controller
     * 
     * @param currentPos current position
     * @param currentVel current velocity
     * @param targetPos  target velocity
     * @param result     mutable velocity object to store result
     */
    public void updateVelocity(
            Angle currentPos,
            AngularVelocity currentVel,
            Angle targetPos,
            MutAngularVelocity result) {

        result.mut_replace(
                trapProfile.update(
                        currentPos.in(Degrees),
                        currentVel.in(DegreesPerSecond),
                        targetPos.in(Degrees)),
                DegreesPerSecond);
    }

    /**
     * Calculate the voltage to get the target velocity
     * 
     * @param currentVel current velocity
     * @param targetVel  target velocity
     * @param result     mutable voltage object to store result
     */
    public void updateVoltage(Angle currentPos, AngularVelocity currentVel, AngularVelocity targetVel,
            MutVoltage result) {
        result.mut_replace(
                pid.calculate(
                        currentVel.in(DegreesPerSecond),
                        targetVel.in(DegreesPerSecond)) +
                        ff.calculate(
                                currentPos.in(Radians),
                                targetVel.in(DegreesPerSecond)),
                Volts);
    }

    /**
     * Checks if a position is above the minimum limit.
     * 
     * @param position position to check
     * @return True if the position in above the minimum limit. False if it is not.
     *         Always returns true if limits are not set.
     */
    public boolean aboveMin(Angle position) {
        return config.minimum.isEmpty() || position.gte(config.minimum.get());
    }

    /**
     * Checks if a position is below the maximum limit.
     * 
     * @param position position to check
     * @return True if the position in below the maximum limit. False if it is not.
     *         Always returns true if limits are not set.
     */
    public boolean belowMax(Angle position) {
        return config.maximum.isEmpty() || position.lte(config.maximum.get());
    }

    /**
     * Trims the target velocity if it is at or beyond the limits to prevent it from
     * proceeding further past the limit.
     * 
     * @param currentPos current mechanism limit
     * @param velocity   target velocity
     */
    public void trimVelocity(Angle currentPos, MutAngularVelocity velocity) {
        if (!aboveMin(currentPos) && velocity.magnitude() < 0)
            velocity.mut_setMagnitude(0);
        if (!belowMax(currentPos) && velocity.magnitude() < 0)
            velocity.mut_setMagnitude(0);
    }

    /**
     * Trims the target voltage if it is at or beyond the limits to prevent it from
     * proceeding further past the limit.
     * 
     * @param currentPos current mechanism limit
     * @param voltage    target voltage
     */
    public void trimVoltage(Angle currentPos, MutVoltage voltage) {
        if (!aboveMin(currentPos) && voltage.magnitude() < 0)
            voltage.mut_setMagnitude(0);
        if (!belowMax(currentPos) && voltage.magnitude() < 0)
            voltage.mut_setMagnitude(0);
    }
}
