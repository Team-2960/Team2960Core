package frc.lib2960.settings;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;

public class SwerveModuleCommonSettings {
    public final Distance wheelDiameter;
    public final Dimensionless driveRatio;
    public final Dimensionless angleRatio;
    public final Dimensionless coupleRatio;

    public final LinearControllerSettings driveCtrlSettings;
    public final AngularControllerSettings angleCtrlSettings;

    public SwerveModuleCommonSettings(
            Distance wheelDiameter,
            Dimensionless driveRatio,
            Dimensionless angleRatio,
            Dimensionless coupleRatio,
            LinearControllerSettings driveCtrlSettings,
            AngularControllerSettings angleCtrlSettings) {

        this.wheelDiameter = wheelDiameter;
        this.driveRatio = driveRatio;
        this.angleRatio = angleRatio;
        this.coupleRatio = coupleRatio;
        this.driveCtrlSettings = driveCtrlSettings;
        this.angleCtrlSettings = angleCtrlSettings;
    }
}
