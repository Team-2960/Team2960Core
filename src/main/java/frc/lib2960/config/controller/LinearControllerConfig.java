package frc.lib2960.config.controller;

import java.util.Optional;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.lib2960.controller.LinearController;

public class LinearControllerConfig {
    public final PIDConfig pidConfig;
    public final FFConfig ffConfig;

    public final Time period;

    public final LinearVelocity maxVel;
    public final LinearAcceleration maxAccel;
    public final LinearAcceleration maxDecel;

    public final Optional<Distance> minimum;
    public final Optional<Distance> maximum;

    LinearControllerConfig(
        Time period, 
        LinearVelocity maxVel,
        LinearAcceleration maxAccel,
        LinearAcceleration maxDecel
    ) {
        this.pidConfig = new PIDConfig(0, 0, 0);
        this.ffConfig = new FFConfig(0, 0);
        this.period = period;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
        this.minimum = Optional.empty();
        this.maximum = Optional.empty();
    }

    LinearControllerConfig(
        Time period, 
        LinearVelocity maxVel,
        LinearAcceleration maxAccel,
        LinearAcceleration maxDecel,
        Distance minimum,
        Distance maximum
    ) {
        this.pidConfig = new PIDConfig(0, 0, 0);
        this.ffConfig = new FFConfig(0, 0);
        this.period = period;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
        this.minimum = Optional.of(minimum);
        this.maximum = Optional.of(maximum);
    }

    LinearControllerConfig(
        PIDConfig pidConfig,
        FFConfig ffConfig,
        Time period, 
        LinearVelocity maxVel,
        LinearAcceleration maxAccel,
        LinearAcceleration maxDecel
    ) {
        this.pidConfig = pidConfig;
        this.ffConfig = ffConfig;
        this.period = period;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
        this.minimum = Optional.empty();
        this.maximum = Optional.empty();
    }

    LinearControllerConfig(
        PIDConfig pidConfig,
        FFConfig ffConfig,
        Time period, 
        LinearVelocity maxVel,
        LinearAcceleration maxAccel,
        LinearAcceleration maxDecel,
        Distance minimum,
        Distance maximum
    ) {
        this.pidConfig = pidConfig;
        this.ffConfig = ffConfig;
        this.period = period;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
        this.minimum = Optional.of(minimum);
        this.maximum = Optional.of(maximum);
    }

    public LinearController getController() {
        return new LinearController(this);
    }
}
