package frc.lib2960.config.controller;

import java.util.Optional;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import frc.lib2960.controller.AngularController;

public class AngularControllerConfig {
    public final PIDConfig pidConfig;
    public final FFConfig ffConfig;

    public final Time period;

    public final AngularVelocity maxVel;
    public final AngularAcceleration maxAccel;
    public final AngularAcceleration maxDecel;

    public final Optional<Angle> minimum;
    public final Optional<Angle> maximum;

    AngularControllerConfig(
            Time period,
            AngularVelocity maxVel,
            AngularAcceleration maxAccel,
            AngularAcceleration maxDecel) {
        this.pidConfig = new PIDConfig(0, 0, 0);
        this.ffConfig = new FFConfig(0, 0);
        this.period = period;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
        this.minimum = Optional.empty();
        this.maximum = Optional.empty();
    }

    AngularControllerConfig(
            Time period,
            AngularVelocity maxVel,
            AngularAcceleration maxAccel,
            AngularAcceleration maxDecel,
            Angle minimum,
            Angle maximum) {
        this.pidConfig = new PIDConfig(0, 0, 0);
        this.ffConfig = new FFConfig(0, 0);
        this.period = period;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
        this.minimum = Optional.of(minimum);
        this.maximum = Optional.of(maximum);
    }

    AngularControllerConfig(
            PIDConfig pidConfig,
            FFConfig ffConfig,
            Time period,
            AngularVelocity maxVel,
            AngularAcceleration maxAccel,
            AngularAcceleration maxDecel) {
        this.pidConfig = pidConfig;
        this.ffConfig = ffConfig;
        this.period = period;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
        this.minimum = Optional.empty();
        this.maximum = Optional.empty();
    }

    AngularControllerConfig(
            PIDConfig pidConfig,
            FFConfig ffConfig,
            Time period,
            AngularVelocity maxVel,
            AngularAcceleration maxAccel,
            AngularAcceleration maxDecel,
            Angle minimum,
            Angle maximum) {
        this.pidConfig = pidConfig;
        this.ffConfig = ffConfig;
        this.period = period;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
        this.minimum = Optional.of(minimum);
        this.maximum = Optional.of(maximum);
    }

    public AngularController getController() {
        return new AngularController(this);
    }
}
