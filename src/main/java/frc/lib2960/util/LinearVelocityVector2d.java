package frc.lib2960.util;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;

public class LinearVelocityVector2d {
    public final MutLinearVelocity xVel = MetersPerSecond.mutable(0);
    public final MutLinearVelocity yVel = MetersPerSecond.mutable(0);

    public LinearVelocityVector2d() {}

    public LinearVelocityVector2d(LinearVelocity xVel, LinearVelocity yVel) {
        this.xVel.mut_replace(xVel);
        this.xVel.mut_replace(yVel);
    }
}
