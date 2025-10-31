package frc.lib2960.controller;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;

public class AngularControllerConfig {
    /** PID gains for the controller. kP, kI, & kD default to zero. */
    public PIDConfig pidConfig = new PIDConfig(0, 0, 0);
    /** Feed Forward gains for the controller. kS, kV, kG, & kA default to zero. */
    public FFConfig ffConfig = new FFConfig(0, 0);

    /** Update period of the controller. Defaults to 20ms. */
    public Time period = Seconds.of(0.02);

    /**
     * Maximum velocity of the controller. Defaults to Positive Infinity Radians per
     * Second.
     */
    public AngularVelocity maxVel = RadiansPerSecond.of(Double.POSITIVE_INFINITY);
    
    /**
     * Maximum acceleration of the controller. Defaults to Positive Infinity Radians
     * per Second per Second.
     */
    public AngularAcceleration maxAccel = RadiansPerSecondPerSecond.of(Double.POSITIVE_INFINITY);

    /**
     * Maximum deceleration of the controller. Defaults to Positive Infinity Radians
     * per Second per Second.
     */
    public AngularAcceleration maxDecel = RadiansPerSecondPerSecond.of(Double.POSITIVE_INFINITY);

    /**
     * Minimum position of the controller. if empty, system is assumed to be
     * continuous. Defaults to empty.
     */
    public Optional<Angle> minimum = Optional.empty();
    /**
     * Maximum position of the controller. if empty, system is assumed to be
     * continuous. Defaults to empty.
     */
    public Optional<Angle> maximum = Optional.empty();

    /** Display units for position. Defaults to Degrees. */
    public AngleUnit posUnit = Degrees;

    /** Display units for time. Defaults to Seconds. */
    public TimeUnit timeUnit = Seconds;

    /**
     * Sets the PIDConfig. Default config is kP, kI, and kD are set to zero.
     * 
     * @param pidConfig new PID config
     * @return current config object
     */
    public AngularControllerConfig setPIDConfig(PIDConfig pidConfig) {
        this.pidConfig = pidConfig;
        return this;
    }

    /**
     * Sets the FFConfig. Default config is kS, kV, kG, and kA are set to zero.
     * 
     * @param ffConfig new FF config
     * @return current config object
     */
    public AngularControllerConfig setFFConfig(FFConfig ffConfig) {
        this.ffConfig = ffConfig;
        return this;
    }

    /**
     * Update period. Default is 20ms.
     * 
     * @param period update period
     * @return current config object
     */
    public AngularControllerConfig setPeriod(Time period) {
        this.period = period;
        return this;
    }

    /**
     * Sets the maximum velocity. Default is positive infinity.
     * 
     * @param maxVel maximum velocity
     * @return current config object
     */
    public AngularControllerConfig setMaxVelocity(AngularVelocity maxVel) {
        this.maxVel = maxVel;
        return this;
    }

    /**
     * Sets the maximum acceleration. Default is positive infinity.
     * 
     * @param maxAccel maximum acceleration
     * @return current config object
     */
    public AngularControllerConfig setMaxAccel(AngularAcceleration maxAccel) {
        this.maxAccel = maxAccel;
        return this;
    }

    /**
     * Sets the maximum deceleration. Default is positive infinity.
     * 
     * @param maxDecel maximum deceleration
     * @return current config object
     */
    public AngularControllerConfig setMaxDecel(AngularAcceleration maxDecel) {
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
    public AngularControllerConfig setLimits(Angle minimum, Angle maximum) {
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
    public AngularControllerConfig clearLimits() {
        this.minimum = Optional.empty();
        this.maximum = Optional.empty();
        return this;
    }

    /**
     * Sets the position display units. Defaults to Degrees.
     * 
     * @param unit position display units
     * @return current config object
     */
    public AngularControllerConfig setPosUnits(AngleUnit unit) {
        posUnit = unit;
        return this;
    }

    /**
     * Sets the position display units. Defaults to Degrees.
     * 
     * @param unit position display units
     * @return current config object
     */
    public AngularControllerConfig setTimeUnits(TimeUnit unit) {
        timeUnit = unit;
        return this;
    }

    /**
     * Copies the configuration from another config object
     * 
     * @param other other config object to copy
     * @return current config object
     */
    public AngularControllerConfig copyConfig(AngularControllerConfig other) {
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
     * Generates a new AngularController object from this configuration
     * 
     * @return new AngularController object from this configuration
     */
    public AngularController getController() {
        return new AngularController(this);
    }
}
