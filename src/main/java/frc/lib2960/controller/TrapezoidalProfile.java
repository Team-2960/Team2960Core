package frc.lib2960.controller;

import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class TrapezoidalProfile implements Sendable {
    private double maxVel;
    private double maxAccel;
    private double maxDecel;

    private double targetPos = 0;
    private double currentPos = 0;
    private double currentVel = 0;
    private double error = 0;
    private double sign = 0;
    private double targetVel = 0;
    private double maxAccelVel = 0;
    private double maxDecelVel = 0;

    private final Time period;

    private final Unit posUnit;
    private final TimeUnit timeUnit;

    /**
     * Constructor. All values must be in the same distance units (i.e. meters,
     * feet, inches, degrees, radians). All values must have the same time units
     * (i.e. seconds, minutes, hours).
     * 
     * @param maxVel   Maximum velocity of the mechanism
     * @param maxAccel Maximum acceleration to get to travel speed
     * @param maxDecel Maximum deceleration to get from travel speed to stop speed
     * @param period   Time period of the controller update
     * @param posUnit  Unit of measurement for the position of the value being
     *                 controlled
     * @param timeUnit Time unit of measurement for the velocity and acceleration of the controller
     *                 update
     */
    public TrapezoidalProfile(double maxVel, double maxAccel, double maxDecel, Time period, Unit posUnit,
            TimeUnit timeUnit) {

        // TODO Improve units library integration
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
        this.period = period;
        this.posUnit = posUnit;
        this.timeUnit = timeUnit;
    }

    /**
     * Calculates the next target speed to get to a target position. All values must
     * be in the same distance units (i.e. meters, feet, inches, degrees, radians).
     * All values must have the same time units (i.e. seconds, minutes, hours).
     * 
     * @param currentPos current mechanism position
     * @param currentVel current mechanism velocity
     * @param targetPos  target mechanism position
     */
    public double update(double currentPos, double currentVel, double targetPos) {
        this.currentPos = currentPos;
        this.currentVel = currentVel;
        this.targetPos = targetPos;

        error = currentPos - targetPos;
        sign = (error > 0 ? 1 : -1);

        // Set default target speed to max velocity
        targetVel = maxVel;
        maxAccelVel = Math.abs(currentVel + maxAccel * period.in(timeUnit));
        maxDecelVel = error * maxDecel / (2 * maxVel);

        // Check if max acceleration keeps velocity lower than max velocity
        targetVel = Math.min(maxAccelVel, targetVel);

        // Check if max deceleration keeps the velocity lower than max velocity
        targetVel = Math.min(maxDecelVel, targetVel);

        return sign * targetVel;
    }

    /**
     * Implements sendable initialization
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        String posUnitStr = "(" + posUnit.symbol() + ")";
        String velUnitStr = "(" + posUnit.symbol() + "per" + timeUnit.symbol() + ")";
        String accelUnitStr = "(" + posUnit.symbol() + "per" + timeUnit.symbol() + "^2)";

        builder.setSmartDashboardType("lib2960 Trapezoidal Profile");
        builder.addDoubleProperty("Max Velocity " + velUnitStr, () -> maxVel, (value) -> maxVel = value);
        builder.addDoubleProperty("Max Acceleration " + accelUnitStr, () -> maxAccel, (value) -> maxAccel = value);
        builder.addDoubleProperty("Max Deceleration " + accelUnitStr, () -> maxDecel, (value) -> maxDecel = value);
        builder.addDoubleProperty("Current Pos " + posUnitStr, () -> currentPos, null);
        builder.addDoubleProperty("Current Vel " + velUnitStr, () -> currentVel, null);
        builder.addDoubleProperty("Target Pos " + posUnitStr, () -> targetPos, null);
        builder.addDoubleProperty("Error " + posUnitStr, () -> error, null);
        builder.addDoubleProperty("Sign", () -> sign, null);
        builder.addDoubleProperty("Target Vel " + velUnitStr, () -> targetVel, null);
        builder.addDoubleProperty("Max Accel Vel " + velUnitStr, () -> maxAccelVel, null);
        builder.addDoubleProperty("Max Decel Vel " + velUnitStr, () -> maxDecelVel, null);
    }
}