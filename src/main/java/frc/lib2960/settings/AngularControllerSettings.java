package frc.lib2960.settings;

import java.util.Optional;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import frc.lib2960.controllers.AngularController;

public class AngularControllerSettings {
    public final PIDSettings pidSettings;
    public final FFSettings ffSettings;

    public final Time period;

    public final AngularVelocity maxVel;
    public final AngularAcceleration maxAccel;
    public final AngularAcceleration maxDecel;

    public final Optional<Angle> minimum;
    public final Optional<Angle> maximum;


    AngularControllerSettings(
        PIDSettings pidSettings,
        FFSettings ffSettings,
        Time period, 
        AngularVelocity maxVel,
        AngularAcceleration maxAccel,
        AngularAcceleration maxDecel
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

    AngularControllerSettings(
        PIDSettings pidSettings,
        FFSettings ffSettings,
        Time period, 
        AngularVelocity maxVel,
        AngularAcceleration maxAccel,
        AngularAcceleration maxDecel,
        Angle minimum,
        Angle maximum
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
    
    public AngularController getController() {
        return new AngularController(this);
    }
}
