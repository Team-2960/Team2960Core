package frc.lib2960.controllers;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib2960.settings.AngularControllerSettings;

public class AngularController {
    public final AngularControllerSettings settings;
    private final TrapezoidalProfile trapProfile;
    private final PIDController pid;
    private final ElevatorFeedforward ff;

    public AngularController(AngularControllerSettings settings) {
        this.settings = settings;
        trapProfile = new TrapezoidalProfile(
                settings.maxVel.in(DegreesPerSecond),
                settings.maxAccel.in(DegreesPerSecondPerSecond),
                settings.maxDecel.in(DegreesPerSecondPerSecond),
                settings.period.in(Seconds));
        pid = settings.pidSettings.getPIDController();
        ff = settings.ffSettings.getElevatorFF();
    }

    public AngularVelocity updatePosition(Angle currentPos, AngularVelocity currentVel, Angle targetPos) {
        return DegreesPerSecond.of(
                trapProfile.update(
                        currentPos.in(Degrees),
                        currentVel.in(DegreesPerSecond),
                        targetPos.in(Degrees)));
    }

    public Voltage updateVelocity(AngularVelocity currentVel, AngularVelocity targetVel) {
        return Volts.of(
                pid.calculate(
                        currentVel.in(DegreesPerSecond),
                        targetVel.in(DegreesPerSecond)) +
                ff.calculate(targetVel.in(DegreesPerSecond)));
    }
}
