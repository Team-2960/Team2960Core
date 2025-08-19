package frc.lib2960.settings;

import java.util.Optional;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.lib2960.controllers.LinearController;

public class LinearControllerSettings {
    public final PIDSettings pidSettings;
    public final FFSettings ffSettings;

    public final Time period;

    public final LinearVelocity maxVel;
    public final LinearAcceleration maxAccel;
    public final LinearAcceleration maxDecel;

    public final Optional<Distance> minimum;
    public final Optional<Distance> maximum;

    LinearControllerSettings(
        PIDSettings pidSettings,
        FFSettings ffSettings,
        Time period, 
        LinearVelocity maxVel,
        LinearAcceleration maxAccel,
        LinearAcceleration maxDecel
    ) {
        this.pidSettings = pidSettings;
        this.ffSettings = ffSettings;
        this.period = period;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
        this.minimum = Optional.empty();
        this.maximum = Optional.empty();
    }

    LinearControllerSettings(
        PIDSettings pidSettings,
        FFSettings ffSettings,
        Time period, 
        LinearVelocity maxVel,
        LinearAcceleration maxAccel,
        LinearAcceleration maxDecel,
        Distance minimum,
        Distance maximum
    ) {
        this.pidSettings = pidSettings;
        this.ffSettings = ffSettings;
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
