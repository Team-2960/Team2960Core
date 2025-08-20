package frc.lib2960.config;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;

public class SwerveDriveBaseConfig {
    public final LinearControllerConfig linearCtrlConfig;
    public final AngularControllerConfig angleCtrlConfig;
    public final Time period;
    public final Vector<N3> stateStd;
    public final Vector<N3> visionStd;

    public SwerveDriveBaseConfig(
            LinearControllerConfig linearCtrlConfig,
            AngularControllerConfig angleCtrlConfig,
            Time period) {

        this(
                linearCtrlConfig,
                angleCtrlConfig,
                period,
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    }

    public SwerveDriveBaseConfig(
            LinearControllerConfig linearCtrlConfig,
            AngularControllerConfig angleCtrlConfig,
            Time period,
            Vector<N3> stateStd,
            Vector<N3> visionStd) {

        this.linearCtrlConfig = linearCtrlConfig;
        this.angleCtrlConfig = angleCtrlConfig;
        this.period = period;
        this.stateStd = stateStd;
        this.visionStd = visionStd;
    }
}
