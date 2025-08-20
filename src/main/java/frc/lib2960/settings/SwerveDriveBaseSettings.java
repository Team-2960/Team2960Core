package frc.lib2960.settings;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;

public class SwerveDriveBaseSettings {
    public final LinearControllerSettings linearCtrlSettings;
    public final AngularControllerSettings angleCtrlSettings;
    public final Time period;
    public final Vector<N3> stateStd;
    public final Vector<N3> visionStd;

    public SwerveDriveBaseSettings(
            LinearControllerSettings linearCtrlSettings,
            AngularControllerSettings angleCtrlSettings,
            Time period) {

        this(
                linearCtrlSettings,
                angleCtrlSettings,
                period,
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    }

    public SwerveDriveBaseSettings(
            LinearControllerSettings linearCtrlSettings,
            AngularControllerSettings angleCtrlSettings,
            Time period,
            Vector<N3> stateStd,
            Vector<N3> visionStd) {

        this.linearCtrlSettings = linearCtrlSettings;
        this.angleCtrlSettings = angleCtrlSettings;
        this.period = period;
        this.stateStd = stateStd;
        this.visionStd = visionStd;
    }
}
