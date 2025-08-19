package frc.lib2960.controllers;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.lib2960.settings.PIDSettings;

public abstract class ControllerBase {
    public enum LimitType {VOLTAGE, RATE}

    private final PIDController pid;

    private final Time period;

    private final MutVoltage voltCalc = Volts.mutable(0);

    private final LimitType limitType;


    protected ControllerBase(PIDSettings pidSettings, Time period) {
        this.pid = pidSettings.getPIDController();
        this.period = period;
        this.limitType = LimitType.VOLTAGE;
    }
    
    protected ControllerBase(PIDSettings pidSettings, Time period, LimitType limitType) {
        this.pid = pidSettings.getPIDController();
        this.period = period;
        this.limitType = limitType;
    }

    public Voltage updatePosition(double currentPos, double currentVel, double targetPos) {
        double error = currentPos - targetPos;
        double sign = (error > 0 ? 1 : -1); 

        // Set default target speed to max velocity
        double maxVel = maxVel();
        double targetVel =  maxVel();

        // Check if max acceleration keeps velocity lower than max velocity
        targetVel = Math.min(Math.abs(currentVel + sign * maxAccel() * period.in(Seconds)), targetVel);
        
        // Check if max deceleration keeps the velocity lower than max velocity
        targetVel = Math.min(maxVel * error / rampDownDist(), targetVel);

        return updateVelocity(currentPos, currentVel, targetVel);
    }

    public Voltage updateVelocity(double currentPos, double currentVel, double targetVel) {
        voltCalc.mut_replace(pid.calculate(targetVel, targetVel), Volts);
        voltCalc.plus(updateFF(currentPos, targetVel));
        return voltCalc;
    }


    protected abstract Voltage updateFF(double currentPos, double targetVel);
    protected abstract double maxVel();
    protected abstract double maxAccel();
    protected abstract double maxDecel();
    protected abstract double minPos();
    protected abstract double maxPos();

    private double rampDownDist() {
        return 2 * Math.pow(maxAccel(), 2) / maxDecel();
    }
}
