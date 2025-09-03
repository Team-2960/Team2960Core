package frc.lib2960.config.subsystem;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import frc.lib2960.config.controller.AngularControllerConfig;
import frc.lib2960.config.controller.LinearControllerConfig;

public class SwerveModuleCommonConfig {
    /** Diameter of the drive wheel. */
    public Distance wheelDiameter;
    /** Radius of the drive wheel */
    public Distance wheelRadius;
    /** Circumfrance of the drive wheel */
    public Distance wheelCircumference;

    /** Drive motor gear ratio */
    public double driveRatio;
    /** Angle motor gear ratio */
    public double angleRatio;
    /**
     * Gear ratio for motion induced in the drive wheel by the angle motor. Defaults
     * to 0.
     */
    public double coupleRatio = 0;

    /** Current required to slip the deive wheels. Defaults to 120A. */
    public Current slipCurrent = Amps.of(120);
    /** Maximum allowed current on the drive motors. Defaults to 80A. */
    public Current maxDriveCurrent = Amps.of(80);
    /** Maximum allowed current on the angle motors. Defaults to 60A. */
    public Current maxAngleCurrent = Amps.of(60);

    /** Drive wheel controller configuration. */
    public LinearControllerConfig driveCtrlConfig = new LinearControllerConfig();
    /** Module angle controller configuration */
    public AngularControllerConfig angleCtrlConfig = new AngularControllerConfig();

    /**
     * Constructor
     * 
     * @param wheelDiameter Diameter of the drive wheel
     * @param driveRatio    Drive motor gear ratio
     * @param angleRatio    Angle motor gear ratio
     */
    public SwerveModuleCommonConfig(
            Distance wheelDiameter,
            double driveRatio,
            double angleRatio) {

        this.wheelDiameter = wheelDiameter;
        this.wheelRadius = wheelDiameter.div(2);
        this.wheelCircumference = wheelDiameter.times(Math.PI);

        this.driveRatio = driveRatio;
        this.angleRatio = angleRatio;
    }

    /**
     * Sets the couple ratio. Defaults to 0.
     * 
     * @param ratio couple ratio
     * @return current configuration object
     */
    public SwerveModuleCommonConfig setCoupleRatio(double ratio) {
        this.coupleRatio = ratio;
        return this;
    }

    /**
     * Sets the slip current. Defaults to 120A.
     * 
     * @param current slip current
     * @return current configuration object
     */
    public SwerveModuleCommonConfig setSlipCurrent(Current current) {
        this.slipCurrent = current;
        return this;
    }

    /**
     * Sets the max drive current. Defaults to 80A.
     * 
     * @param current max drive current
     * @return current configuration object
     */
    public SwerveModuleCommonConfig setMaxDriveCurrent(Current current) {
        this.maxDriveCurrent = current;
        return this;
    }

    /**
     * Sets the max angle current. Defaults to 60A.
     * 
     * @param current max angle current
     * @return current configuration object
     */
    public SwerveModuleCommonConfig setMaxAngleCurrent(Current current) {
        this.maxAngleCurrent = current;
        return this;
    }
}
