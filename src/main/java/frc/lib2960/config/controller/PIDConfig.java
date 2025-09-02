package frc.lib2960.config.controller;

import edu.wpi.first.math.controller.PIDController;

public class PIDConfig {
    // TODO Implement using units library
    public double kP;
    public double kI;
    public double kD;

    public PIDConfig(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public PIDController getPIDController(){
        return new PIDController(kP, kI, kD);
    }
}
