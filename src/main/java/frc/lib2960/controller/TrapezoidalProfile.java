package frc.lib2960.controller;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class TrapezoidalProfile implements Sendable {
    private double maxVel;
    private double maxAccel;
    private double maxDecel;

    private final double period;

    /**
     * Constructor. All values must be in the same distance units (i.e. meters,
     * feet, inches, degrees, radians). All values must have the same time units
     * (i.e. seconds, minutes, hours).
     * 
     * @param maxVel   Maximum velocity of the mechanism
     * @param maxAccel Maximum acceleration to get to travel speed
     * @param maxDecel Maximum deceleration to get from travel speed to stop speed
     * @param period   Time period of the controller update
     */
    public TrapezoidalProfile(double maxVel, double maxAccel, double maxDecel, double period) {
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxDecel = maxDecel;
        this.period = period;
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
        double error = currentPos - targetPos;
        double sign = (error > 0 ? 1 : -1);

        // Set default target speed to max velocity
        double maxVel = this.maxVel;
        double targetVel = maxVel;

        // Check if max acceleration keeps velocity lower than max velocity
        targetVel = Math.min(Math.abs(currentVel + sign * maxAccel * period), targetVel);

        // Check if max deceleration keeps the velocity lower than max velocity
        targetVel = Math.min(maxVel * error / (2 * Math.pow(maxVel, 2) / maxDecel), targetVel);

        return targetVel;
    }

    /**
     * Implements sendable initialization
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("lib2960 Trapezoidal Profile");
        builder.addDoubleProperty("Max Velocity", () -> this.maxVel, (maxVel) -> this.maxVel = maxVel);
        builder.addDoubleProperty("Max Acceleration", () -> this.maxAccel, (maxAccel) -> this.maxAccel = maxAccel);
        builder.addDoubleProperty("Max Deceleration", () -> this.maxDecel, (maxDecel) -> this.maxDecel = maxDecel);
    }


    public void setMaxVel(double maxVel) {
        this.maxVel = maxVel;
    }


    public void setMaxAccel(double maxAccel) {
        this.maxVel = maxAccel;
    }


    public void setMaxDecel(double maxDecel) {
        this.maxDecel = maxDecel;
    }

    public double getMaxVel(){
        return maxVel;
    }

    public double getMaxAccel(){
        return maxAccel;
    }

    public double getMaxDecel(){
        return maxDecel;
    }
}