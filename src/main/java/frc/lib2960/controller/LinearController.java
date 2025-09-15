package frc.lib2960.controller;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib2960.config.controller.LinearControllerConfig;

public class LinearController implements Sendable {
    public final LinearControllerConfig config;
    private final TrapezoidalProfile trapProfile;
    private final PIDController pid;
    private final ElevatorFeedforward ff;

    /**
     * Constructor
     * 
     * @param config Controller config
     */
    public LinearController(LinearControllerConfig config) {
        this.config = config;
        trapProfile = new TrapezoidalProfile(
                config.maxVel.in(MetersPerSecond),
                config.maxAccel.in(MetersPerSecondPerSecond),
                config.maxDecel.in(MetersPerSecondPerSecond),
                config.period.in(Seconds));
        pid = config.pidConfig.getPIDController();
        ff = config.ffConfig.getElevatorFF();
    }

    /**
     * Implements sendable initialization
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        trapProfile.initSendable(builder);
        pid.initSendable(builder);
        builder.addDoubleProperty("FeedForward kS", ff::getKs, ff::setKs);
        builder.addDoubleProperty("FeedForward kV", ff::getKv, ff::setKv);
        builder.addDoubleProperty("FeedForward kG", ff::getKg, ff::setKg);
        builder.addDoubleProperty("FeedForward kA", ff::getKa, ff::setKa);
        builder.setSmartDashboardType("LinearController");
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
            Distance currentPos,
            LinearVelocity currentVel,
            Distance targetPos,
            MutLinearVelocity result) {

        result.mut_replace(
                trapProfile.update(
                        currentPos.in(Meters),
                        currentVel.in(MetersPerSecond),
                        targetPos.in(Meters)),
                MetersPerSecond);
    }

    /**
     * Calculate the voltage to get the target velocity
     * 
     * @param currentVel current velocity
     * @param targetVel  target velocity
     * @param result     mutable voltage object to store result
     */
    public void updateVoltage(LinearVelocity currentVel, LinearVelocity targetVel, MutVoltage result) {
        result.mut_replace(
                pid.calculate(
                        currentVel.in(MetersPerSecond),
                        targetVel.in(MetersPerSecond)) +
                        ff.calculate(targetVel.in(MetersPerSecond)),
                Volts);
    }

    /**
     * Checks if a position is above the minimum limit.
     * 
     * @param position position to check
     * @return True if the position in above the minimum limit. False if it is not.
     *         Always returns true if limits are not set.
     */
    public boolean aboveMin(Distance position) {
        return config.minimum.isEmpty() || position.gte(config.minimum.get());
    }

    /**
     * Checks if a position is below the maximum limit.
     * 
     * @param position position to check
     * @return True if the position in below the maximum limit. False if it is not.
     *         Always returns true if limits are not set.
     */
    public boolean belowMax(Distance position) {
        return config.maximum.isEmpty() || position.lte(config.maximum.get());
    }

    /**
     * Trims the target velocity if it is at or beyond the limits to prevent it from
     * proceeding further past the limit.
     * 
     * @param currentPos current mechanism limit
     * @param velocity   target velocity
     */
    public void trimVelocity(Distance currentPos, MutLinearVelocity velocity) {
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
    public void trimVoltage(Distance currentPos, MutVoltage voltage) {
        if (!aboveMin(currentPos) && voltage.magnitude() < 0)
            voltage.mut_setMagnitude(0);
        if (!belowMax(currentPos) && voltage.magnitude() < 0)
            voltage.mut_setMagnitude(0);
    }
}
