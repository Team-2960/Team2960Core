package frc.lib2960.controller;

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
        this.currentPos = currentPos;
        this.currentVel = currentVel;
        this.targetPos = targetPos;

        error = currentPos - targetPos;
        sign = (error > 0 ? 1 : -1);

        // Set default target speed to max velocity
        targetVel = maxVel;
        maxAccelVel = Math.abs(currentVel + maxAccel * period);
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
        builder.setSmartDashboardType("lib2960 Trapezoidal Profile");
        builder.addDoubleProperty("Max Velocity", this::getMaxVel, this::setMaxVel);
        builder.addDoubleProperty("Max Acceleration", this::getMaxAccel, this::setMaxAccel);
        builder.addDoubleProperty("Max Deceleration", this::getMaxDecel, this::setMaxDecel);
        builder.addDoubleProperty("Current Pos", () -> currentPos, null);
        builder.addDoubleProperty("Current Vel", () -> currentVel, null);
        builder.addDoubleProperty("Target Pos", () -> targetPos, null);
        builder.addDoubleProperty("Error", () -> error, null);
        builder.addDoubleProperty("Sign", () -> sign, null);
        builder.addDoubleProperty("Target Vel", () -> targetVel, null);
        builder.addDoubleProperty("Max Accel Vel", () -> maxAccelVel, null);
        builder.addDoubleProperty("Max Decel Vel", () -> maxDecelVel, null);
    }

    /**
     * Sets the maximum velocity.
     * 
     * @param maxVel Maximum velocity
     */
    public void setMaxVel(double maxVel) {
        this.maxVel = maxVel;
    }

    /**
     * Sets the maximum acceleration.
     * 
     * @param maxAccel Maximum acceleration
     */
    public void setMaxAccel(double maxAccel) {
        this.maxAccel = maxAccel;
    }

    /**
     * Sets the maximum deceleration.
     * 
     * @param maxDecel Maximum deceleration
     */
    public void setMaxDecel(double maxDecel) {
        this.maxDecel = maxDecel;
    }

    /**
     * Gets the maximum velocity
     * 
     * @return maximum velocity
     */
    public double getMaxVel() {
        return maxVel;
    }

    /**
     * Gets the maximum acceleration
     * 
     * @return maximum acceleration
     */
    public double getMaxAccel() {
        return maxAccel;
    }

    /**
     * Gets the maximum deceleration
     * 
     * @return maximum deceleration
     */
    public double getMaxDecel() {
        return maxDecel;
    }
}