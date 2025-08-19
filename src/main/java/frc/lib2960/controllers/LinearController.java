package frc.lib2960.controllers;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib2960.settings.LinearControllerSettings;

public class LinearController {
    public final LinearControllerSettings settings;
    private final TrapezoidalProfile trapProfile;
    private final PIDController pid;
    private final ElevatorFeedforward ff;

    public LinearController(LinearControllerSettings settings) {
        this.settings = settings;
        trapProfile = new TrapezoidalProfile(
                settings.maxVel.in(MetersPerSecond),
                settings.maxAccel.in(MetersPerSecondPerSecond),
                settings.maxDecel.in(MetersPerSecondPerSecond),
                settings.period.in(Seconds));
        pid = settings.pidSettings.getPIDController();
        ff = settings.ffSettings.getElevatorFF();
    }

    public LinearVelocity updatePosition(Distance currentPos, LinearVelocity currentVel, Distance targetPos) {
        return MetersPerSecond.of(
                trapProfile.update(
                        currentPos.in(Meters),
                        currentVel.in(MetersPerSecond),
                        targetPos.in(Meters)));
    }

    public Voltage updateVelocity(LinearVelocity currentVel, LinearVelocity targetVel) {
        return Volts.of(
                pid.calculate(
                        currentVel.in(MetersPerSecond),
                        targetVel.in(MetersPerSecond)) +
                ff.calculate(targetVel.in(MetersPerSecond)));
    }
}
