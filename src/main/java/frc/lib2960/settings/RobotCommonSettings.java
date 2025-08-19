package frc.lib2960.settings;

import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;

public class RobotCommonSettings {
    public Time updatePeriod = Seconds.of(.020);

    public Mass robotMass = Pounds.of(120);
    public Mass bumperMass = Pounds.of(15);  
    public Mass batteryMass = Pounds.of(12.89);
    public Mass totalMass = Pounds.of(robotMass.in(Pounds) + bumperMass.in(Pounds) + batteryMass.in(Pounds));
    
}
