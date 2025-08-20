package frc.lib2960.config;

import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;

public class NavXSwerveDriveConfig extends SwerveDriveBaseConfig {
    public final NavXComType comType;

    public NavXSwerveDriveConfig(
            LinearControllerConfig linearCtrlConfig,
            AngularControllerConfig angleCtrlConfig,
            Time period,
            NavXComType comType) {

        super(linearCtrlConfig, angleCtrlConfig, period);
        this.comType = comType;

    }

    public NavXSwerveDriveConfig(
            LinearControllerConfig linearCtrlConfig,
            AngularControllerConfig angleCtrlConfig,
            Time period,
            Vector<N3> stateStd,
            Vector<N3> visionStd,
            NavXComType comType) {
        super(linearCtrlConfig, angleCtrlConfig, period, stateStd, visionStd);
        this.comType = comType;
    }
}
