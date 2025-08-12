package frc.lib2960.settings;

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
}
