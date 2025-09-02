package frc.lib2960.subsystem.drivetrain;

import edu.wpi.first.units.measure.Time;

public interface NonHolonomicDrivetrain extends Drivetrain{
    public Time getUpdatePeriod();
}
