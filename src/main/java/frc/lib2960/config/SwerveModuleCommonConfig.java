package frc.lib2960.config;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

public class SwerveModuleCommonConfig {
    public final Distance wheelDiameter;
    public final Distance wheelRadius;
    public final Distance wheelCircumference;

    public final double driveRatio;
    public final double angleRatio;
    public final double coupleRatio;

    public final Current slipCurrent;
    public final Current maxDriveCurrent;
    public final Current maxAngleCurrent;

    public final LinearControllerConfig driveCtrlConfig;
    public final AngularControllerConfig angleCtrlConfig;

    public SwerveModuleCommonConfig(
            Distance wheelDiameter,
            double driveRatio,
            double angleRatio,
            double coupleRatio,
            Current slipCurrent,
            Current maxDriveCurrent,
            Current maxAngleCurrent,
            LinearControllerConfig driveCtrlConfig,
            AngularControllerConfig angleCtrlConfig) {

        this.wheelDiameter = wheelDiameter;
        this.wheelRadius = wheelDiameter.div(2);
        this.wheelCircumference = wheelDiameter.times(Math.PI);

        this.driveRatio = driveRatio;
        this.angleRatio = angleRatio;
        this.coupleRatio = coupleRatio;

        this.slipCurrent = slipCurrent;
        this.maxDriveCurrent = maxDriveCurrent;
        this.maxAngleCurrent = maxAngleCurrent;

        this.driveCtrlConfig = driveCtrlConfig;
        this.angleCtrlConfig = angleCtrlConfig;
    }
}
