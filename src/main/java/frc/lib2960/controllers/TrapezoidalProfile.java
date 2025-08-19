package frc.lib2960.controllers;

public class TrapezoidalProfile {
    private final double maxVel;
    private final double maxAccel;
    private final double rampDownDist;

    private final double period;

    /**
     * Constructor. All values must be in the same distance units (i.e. meters,
     * feet, inches, degrees, radians). All values must have the same time units
     * (i.e. seconds, minutes, hours).
     * 
     * @param maxVel Maximum velocity of the mechanism
     * @param maxAccel Maximum acceleration to get to travel speed
     * @param maxDecel Maximum deceleration to get from travel speed to stop speed
     * @param period   Time period of the controller update
     */
    public TrapezoidalProfile(double maxVel, double maxAccel, double maxDecel, double period) {
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.period = period;

        this.rampDownDist = 2 * Math.pow(maxVel, 2) / maxDecel;
    }

    /**
     * Calculates the next target speed to get to a target position. All values must
     * be in the same distance units (i.e. meters, feet, inches, degrees, radians).
     * All values must have the same time units (i.e. seconds, minutes, hours).
     * 
     * @param currentPos current mechanism position
     * @param currentVel current mechanism velocity
     * @param targetPos target mechanism position
     */
    public double update(double currentPos, double currentVel, double targetPos) {
        double error = currentPos - targetPos;
        double sign = (error > 0 ? 1 : -1); 

        // Set default target speed to max velocity
        double maxVel = this.maxVel;
        double targetVel =  maxVel;

        // Check if max acceleration keeps velocity lower than max velocity
        targetVel = Math.min(Math.abs(currentVel + sign * maxAccel * period), targetVel);
        
        // Check if max deceleration keeps the velocity lower than max velocity
        targetVel = Math.min(maxVel * error / rampDownDist, targetVel);
        
        return targetVel;
    }
}