package frc.lib2960.controllers;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import frc.lib2960.settings.LinearControllerSettings;

public class LinearController extends ControllerBase {
    public final LinearControllerSettings settings;
    private final ElevatorFeedforward ff;
    private final MutVoltage voltCalc = Volts.mutable(0);

    public LinearController(LinearControllerSettings settings) {
        super(settings.pidSettings, settings.period);
        this.settings = settings;
        ff = settings.ffSettings.getElevatorFF();
    }

    public Voltage updatePosition(Distance currentPos, LinearVelocity currentVel, Distance targetPos) {
        Voltage result = super.updatePosition(
                currentPos.in(Meters),
                currentVel.in(MetersPerSecond),
                targetPos.in(Meters));


        if(result.magnitude() > 0 && settings.limits.)

        return result;
    }

    public Voltage updateVelocity(LinearVelocity currentVel, LinearVelocity targetVel) {
        return super.updateVelocity(
                0,
                currentVel.in(MetersPerSecond),
                targetVel.in(MetersPerSecond));
    }

    @Override
    protected double maxVel() {
        return settings.maxVel.in(MetersPerSecond);
    }

    @Override
    protected double maxAccel() {
        return settings.maxAccel.in(MetersPerSecondPerSecond);
    }

    @Override
    protected double maxDecel() {
        return settings.maxDecel.in(MetersPerSecondPerSecond);
    }

    @Override
    protected Voltage updateFF(double currentPos, double targetVel) {
        return voltCalc.mut_replace(ff.calculate(targetVel), Volts);
    }

}
