package frc.lib2960.config;

import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;
import frc.lib2960.config.basic.PIDConfig;

public class NavXSwerveDriveConfig extends SwerveDriveBaseConfig {
    public final NavXComType comType;

    public NavXSwerveDriveConfig(
            String name,
            LinearControllerConfig linearCtrlConfig,
            AngularControllerConfig angleCtrlConfig,
            PIDConfig linearPPPID,
            PIDConfig angularPPPID,
            Time period,
            NavXComType comType) {

        super(name, linearCtrlConfig, angleCtrlConfig, linearPPPID, angularPPPID, period);
        this.comType = comType;

    }

    public NavXSwerveDriveConfig(
            String name,
            LinearControllerConfig linearCtrlConfig,
            AngularControllerConfig angleCtrlConfig,
            Time period,
            PIDConfig linearPPPID,
            PIDConfig angularPPPID,
            Vector<N3> stateStd,
            Vector<N3> visionStd,
            NavXComType comType) {
        super(name, linearCtrlConfig, angleCtrlConfig, linearPPPID, angularPPPID, period, stateStd, visionStd);
        this.comType = comType;
    }
}
