package frc.lib2960.settings;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;

public class CTRESwerveDriveSettings extends SwerveDriveBaseSettings{
        public CTRESwerveDriveSettings(
            LinearControllerSettings linearCtrlSettings,
            AngularControllerSettings angleCtrlSettings,
            Time period,
            Vector<N3> stateStd,
            Vector<N3> visionStd) {
        super(linearCtrlSettings, angleCtrlSettings, period, stateStd, visionStd);
    }
}
