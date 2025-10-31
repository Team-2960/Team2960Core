package frc.lib2960.controller;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public class LinearControllerConfig {
    /** PID gains for the controller. kP, kI, & kD default to zero. */
    public PIDConfig pidConfig = new PIDConfig(0, 0, 0);
    /** Feed Forward gains for the controller. kS, kV, kG, & kA default to zero. */
    public FFConfig ffConfig = new FFConfig(0, 0);

    /** Update period of the controller. Defaults to 20ms. */
    public Time period = Seconds.of(0.02);

    /**
     * Maximum velocity of the controller. Defaults to Positive Infinity Meters per
     * Second.
     */
    public LinearVelocity maxVel = MetersPerSecond.of(Double.POSITIVE_INFINITY);

    /**
     * Maximum acceleration of the controller. Defaults to Positive Infinity Meters
     * per Second per Second.
     */
    public LinearAcceleration maxAccel = MetersPerSecondPerSecond.of(Double.POSITIVE_INFINITY);

    /**
     * Maximum deceleration of the controller. Defaults to Positive Infinity Meters
     * per Second per Second.
     */
    public LinearAcceleration maxDecel = MetersPerSecondPerSecond.of(Double.POSITIVE_INFINITY);

    /**
     * Minimum position of the controller. if empty, system is assumed to be
     * continuous. Defaults to empty.
     */
    public Optional<Distance> minimum = Optional.empty();

    /**
     * Maximum position of the controller. if empty, system is assumed to be
     * continuous. Defaults to empty.
     */
    public Optional<Distance> maximum = Optional.empty();

    /** Display units for position. Defaults to Degrees. */
    public DistanceUnit posUnit = Meters;

    /** Display units for time. Defaults to Seconds. */
    public TimeUnit timeUnit = Seconds;

    /**
     * Sets the PIDConfig. Default config is kP, kI, and kD are set to zero.
     * 
     * @param pidConfig new PID config
     * @return current config object
     */
    public LinearControllerConfig setPIDConfig(PIDConfig pidConfig) {
        this.pidConfig = pidConfig;
        return this;
    }

    /**
     * Sets the FFConfig. Default config is kS, kV, kG, and kA are set to zero.
     * 
     * @param ffConfig new FF config
     * @return current config object
     */
    public LinearControllerConfig setFFConfig(FFConfig ffConfig) {
        this.ffConfig = ffConfig;
        return this;
    }

    /**
     * Update period. Default is 20ms.
     * 
     * @param period update period
     * @return current config object
     */
    public LinearControllerConfig setPeriod(Time period) {
        this.period = period;
        return this;
    }

    /**
     * Sets the maximum velocity. Default is positive infinity.
     * 
     * @param maxVel maximum velocity
     * @return current config object
     */
    public LinearControllerConfig setMaxVelocity(LinearVelocity maxVel) {
        this.maxVel = maxVel;
        return this;
    }

    /**
     * Sets the maximum acceleration. Default is positive infinity.
     * 
     * @param maxAccel maximum acceleration
     * @return current config object
     */
    public LinearControllerConfig setMaxAccel(LinearAcceleration maxAccel) {
        this.maxAccel = maxAccel;
        return this;
    }

    /**
     * Sets the maximum deceleration. Default is positive infinity.
     * 
     * @param maxDecel maximum deceleration
     * @return current config object
     */
    public LinearControllerConfig setMaxDecel(LinearAcceleration maxDecel) {
        this.maxDecel = maxDecel;
        return this;
    }

    /**
     * Sets the minimum and maximum position. If not set, the system is assumed to
     * be continuous.
     * 
     * @param minimum minimum position
     * @param maximum maximum position
     * @return current config object
     */
    public LinearControllerConfig setLimits(Distance minimum, Distance maximum) {
        this.minimum = Optional.of(minimum);
        this.maximum = Optional.of(maximum);
        return this;
    }

    /**
     * Clears the position limits. System will be assumed to be continuous.
     * 
     * @param minimum minimum position
     * @param maximum maximum position
     * @return current config object
     */
    public LinearControllerConfig clearLimits() {
        this.minimum = Optional.empty();
        this.maximum = Optional.empty();
        return this;
    }

    /**
     * Sets the position display units. Defaults to Meters.
     * 
     * @param unit position display units
     * @return current config object
     */
    public LinearControllerConfig setPosUnits(DistanceUnit unit) {
        posUnit = unit;
        return this;
    }

    /**
     * Sets the position display units. Defaults to Degrees.
     * 
     * @param unit position display units
     * @return current config object
     */
    public LinearControllerConfig setTimeUnits(TimeUnit unit) {
        timeUnit = unit;
        return this;
    }

    /**
     * Copies the configuration from another config object
     * 
     * @param other other config object to copy
     * @return current config object
     */
    public LinearControllerConfig copyConfig(LinearControllerConfig other) {
        this.pidConfig = other.pidConfig;
        this.ffConfig = other.ffConfig;

        this.period = other.period;

        this.maxVel = other.maxVel;
        this.maxAccel = other.maxAccel;
        this.maxDecel = other.maxDecel;

        this.minimum = other.minimum;
        this.maximum = other.maximum;

        return this;
    }

    /**
     * Generates a new LinearController object from this configuration
     * 
     * @return new LinearController object from this configuration
     */
    public LinearController getController() {
        return new LinearController(this);
    }
}
