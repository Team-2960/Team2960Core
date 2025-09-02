package frc.lib2960.config.controller;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.lib2960.controller.LinearController;

public class LinearControllerConfig {
    private PIDConfig pidConfig = new PIDConfig(0, 0, 0);
    private FFConfig ffConfig = new FFConfig(0, 0);

    private Time period = Seconds.of(0.02);

    private LinearVelocity maxVel = MetersPerSecond.of(Double.POSITIVE_INFINITY);
    private LinearAcceleration maxAccel = MetersPerSecondPerSecond.of(Double.POSITIVE_INFINITY);
    private LinearAcceleration maxDecel = MetersPerSecondPerSecond.of(Double.POSITIVE_INFINITY);

    private Optional<Distance> minimum = Optional.empty();
    private Optional<Distance> maximum = Optional.empty();

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
     * @param maxAccel  maximum acceleration
     * @return current config object
     */
    public LinearControllerConfig setMaxAccel(LinearAcceleration maxAccel) {
        this.maxAccel = maxAccel;
        return this;
    }

    /**
     * Sets the maximum deceleration. Default is positive infinity.
     * @param maxDecel  maximum deceleration
     * @return current config object
     */
    public LinearControllerConfig setMaxDecel(LinearAcceleration maxDecel) {
        this.maxDecel = maxDecel;
        return this;
    }

    /**
     * Sets the minimum and maximum position. If not set, the system is assumed to be continuous.
     * @param minimum   minimum position
     * @param maximum   maximum position
     * @return current config object
     */
    public LinearControllerConfig setLimits(Distance minimum, Distance maximum) {
        this.minimum = Optional.of(minimum);
        this.maximum = Optional.of(maximum);
        return this;
    }

    /**
     * Clears the position limits. System will be assumed to be continuous.
     * @param minimum   minimum position
     * @param maximum   maximum position
     * @return current config object
     */
    public LinearControllerConfig clearLimits() {
        this.minimum = Optional.empty();
        this.maximum = Optional.empty();
        return this;
    }

    /**
     * Copies the configuration from another config object
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
     * Gets the set PIDConfig 
     * @return  set PIDConfig
     */
    public PIDConfig getPIDConfig() {
        return pidConfig;
    }

    /**
     * Gets the set FFConfig 
     * @return  set FFConfig
     */
    public FFConfig getFFConfig() {
        return ffConfig;
    }


    /**
     * Gets the set Period 
     * @return  set Period
     */
    public Time getPeriod() {
        return period;
    }



    /**
     * Gets the set maximum velocity 
     * @return  set Period
     */
    public LinearVelocity getMaxVelocity() {
        return maxVel;
    }


    /**
     * Gets the set maximum acceleration 
     * @return  set Period
     */
    public LinearAcceleration getMaxAccel() {
        return maxAccel;
    }


    /**
     * Gets the set maximum deceleration 
     * @return  set Period
     */
    public LinearAcceleration getMaxDecel() {
        return maxDecel;
    }

    /**
     * Get the minimum position.
     * @return  minimum position. Set to empty is no limits are set.
     */
    public Optional<Distance> getMinimum() {
        return minimum;
    }


    /**
     * Get the maximum position.
     * @return  maximum position. Set to empty is no limits are set.
     */
    public Optional<Distance> getMaximum() {
        return maximum;
    }

    /**
     * Generates a new LinearController object from this configuration
     * @return  new LinearController object from this configuration
     */
    public LinearController getController() {
        return new LinearController(this);
    }
}
