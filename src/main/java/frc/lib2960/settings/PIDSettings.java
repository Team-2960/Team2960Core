package frc.lib2960.settings;

import edu.wpi.first.math.controller.PIDController;

public class PIDSettings {
    // TODO Implement using units library
    public double kP;
    public double kI;
    public double kD;

    public PIDSettings(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public PIDController getPIDController(){
        return new PIDController(kP, kI, kD);
    }
}
