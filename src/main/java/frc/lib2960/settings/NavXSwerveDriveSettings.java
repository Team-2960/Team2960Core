package frc.lib2960.settings;

import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;

public class NavXSwerveDriveSettings extends SwerveDriveBaseSettings {
    public final NavXComType comType;

    public NavXSwerveDriveSettings(
            LinearControllerSettings linearCtrlSettings,
            AngularControllerSettings angleCtrlSettings,
            Time period,
            NavXComType comType) {

        super(linearCtrlSettings, angleCtrlSettings, period);
        this.comType = comType;

    }

    public NavXSwerveDriveSettings(
            LinearControllerSettings linearCtrlSettings,
            AngularControllerSettings angleCtrlSettings,
            Time period,
            Vector<N3> stateStd,
            Vector<N3> visionStd,
            NavXComType comType) {
        super(linearCtrlSettings, angleCtrlSettings, period, stateStd, visionStd);
        this.comType = comType;
    }
}
