package frc.lib2960.config;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import frc.lib2960.config.basic.PIDConfig;

public class SwerveDriveBaseConfig {
    public final String name;
    public final LinearControllerConfig linearCtrlConfig;
    public final AngularControllerConfig angleCtrlConfig;
    public final PIDConfig linearPPPID;
    public final PIDConfig angularPPPID;
    public final Time period;
    public final Vector<N3> stateStd;
    public final Vector<N3> visionStd;

    public SwerveDriveBaseConfig(
            String name,
            LinearControllerConfig linearCtrlConfig,
            AngularControllerConfig angleCtrlConfig,
            PIDConfig linearPPPID,
            PIDConfig angularPPPID,
            Time period) {

        this(
                name,
                linearCtrlConfig,
                angleCtrlConfig,
                linearPPPID,
                angularPPPID,
                period,
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    }

    public SwerveDriveBaseConfig(
            String name,
            LinearControllerConfig linearCtrlConfig,
            AngularControllerConfig angleCtrlConfig,
            PIDConfig linearPPPID,
            PIDConfig angularPPPID,
            Time period,
            Vector<N3> stateStd,
            Vector<N3> visionStd) {

        this.name = name;
        this.linearCtrlConfig = linearCtrlConfig;
        this.angleCtrlConfig = angleCtrlConfig;
        this.linearPPPID = linearPPPID;
        this.angularPPPID = angularPPPID;
        this.period = period;
        this.stateStd = stateStd;
        this.visionStd = visionStd;
    }
}
