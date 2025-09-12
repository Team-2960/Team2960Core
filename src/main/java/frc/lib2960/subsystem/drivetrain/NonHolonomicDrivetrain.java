package frc.lib2960.subsystem.drivetrain;

import edu.wpi.first.units.measure.Time;

public interface NonHolonomicDrivetrain extends Drivetrain {
    /**
     * Gets the update period of the drivetrain
     * 
     * @return update period of the drivetrain
     */
    public Time getUpdatePeriod();
}
