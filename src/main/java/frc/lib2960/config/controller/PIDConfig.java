package frc.lib2960.config.controller;

import edu.wpi.first.math.controller.PIDController;

public class PIDConfig {
    // TODO Implement using units library
    public double kP;
    public double kI;
    public double kD;

    /**
     * Constructor
     * @param kP    Proportional gain
     * @param kI    Integral gain
     * @param kD    Derivative gain
     */
    public PIDConfig(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /**
     * Generates a PID controller object
     * @return  new PID controller object
     */
    public PIDController getPIDController(){
        return new PIDController(kP, kI, kD);
    }
}
