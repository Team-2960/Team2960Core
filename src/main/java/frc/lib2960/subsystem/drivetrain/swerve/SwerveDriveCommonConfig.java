package frc.lib2960.subsystem.drivetrain.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Time;
import frc.lib2960.controller.AngularControllerConfig;
import frc.lib2960.controller.LinearControllerConfig;
import frc.lib2960.controller.PIDConfig;

public class SwerveDriveCommonConfig {
    /** Name of the mechanism */
    public String name = "Drivetrain";

    /** Name of the tab the mechanism will be displayed on */
    public String uiTabName = "Drivetrain";

    /**
     * Config for linear control of the robot base. Defaults to a default
     * LinearControlConfig.
     */
    public LinearControllerConfig linearCtrlConfig = new LinearControllerConfig();
    /**
     * Config for anguler control of the robot base. Defaults to a default
     * AngularControlConfig.
     */
    public AngularControllerConfig angularCtrlConfig = new AngularControllerConfig();

    /**
     * PID Config for linear PathPlanner control. Defaults to kP, kI, & kD set to
     * zero.
     */
    public PIDConfig linearPPPID = new PIDConfig(0, 0, 0);

    /**
     * PID Config for angular PathPlanner control. Defaults to kP, kI, & kD set to
     * zero.
     */
    public PIDConfig angularPPPID = new PIDConfig(0, 0, 0);

    /** Update period of the drive base */
    public Time period = Seconds.of(0.02);

    /**
     * Odometry standard deviation vector. Defaults to [.05, .05,
     * Units.degreesToRadians(5)]
     */
    public Vector<N3> stateStd = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

    /**
     * Default vision standard deviation vector. Defaults to [.5, .5,
     * Units.degreesToRadians(30)]
     */
    public Vector<N3> visionStd = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30));

    /** Position display unit for linear motion. Defaults to Meters */
    public DistanceUnit linPosUnit = Meters;

    /** Time display unit for linear motion. Defaults to Seconds */
    public TimeUnit linTimeUnit = Seconds;

    /** Position display unit for angular motion. Defaults to Degrees */
    public AngleUnit angPosUnit = Degrees;

    /** Time display unit for angular motion. Defaults to Seconds */
    public TimeUnit angTimeUnit = Seconds;

    /**
     * Sets the name of the mechanism. Defaults to "Drivetrain"
     * 
     * @param name name of the mechanism. D
     * @return current configuration object
     */
    public SwerveDriveCommonConfig setName(String name) {
        this.name = name;
        return this;
    }

    /**
     * Sets the name of the tab the telemetry of this mechanism will appears on.
     * Defaults to "Drivetrain".
     * 
     * @param uiTabName name of the tab the telemetry of this mechanism will appears
     *                  on
     * @return current configuration object
     */
    public SwerveDriveCommonConfig setUITabName(String uiTabName) {
        this.uiTabName = uiTabName;
        return this;
    }

    /**
     * Sets the linear controller configuration. defaults to new
     * LinearControllerConfig().
     * 
     * @param linearCtrlConfig new linear controller config
     * @return current configuration object
     */
    public SwerveDriveCommonConfig setLinearControlConfig(LinearControllerConfig linearCtrlConfig) {
        this.linearCtrlConfig = linearCtrlConfig;
        return this;
    }

    /**
     * Sets the angular controller configuration. defaults to new
     * AngularControllerConfig().
     * 
     * @param angularCtrlConfig new angular controller config
     * @return current configuration object
     */
    public SwerveDriveCommonConfig setAngularControlConfig(AngularControllerConfig angularCtrlConfig) {
        this.angularCtrlConfig = angularCtrlConfig;
        return this;
    }

    /**
     * Set the linear PathPlanner PID configuration. Default is kP, kI, and kD are
     * all set to zero,
     * 
     * @param pidConfig linear PathPlanner PID
     * @return current configuration object
     */
    public SwerveDriveCommonConfig setLinearPPPID(PIDConfig pidConfig) {
        this.linearPPPID = pidConfig;
        return this;
    }

    /**
     * Set the angular PathPlanner PID configuration. Default is kP, kI, and kD are
     * all set to zero,
     * 
     * @param pidConfig angular PathPlanner PID
     * @return current configuration object
     */
    public SwerveDriveCommonConfig setAngularPPPID(PIDConfig pidConfig) {
        this.angularPPPID = pidConfig;
        return this;
    }

    /**
     * Sets the update period. Default is set to 20 ms.
     * 
     * @param period update period
     * @return current configuration object
     */
    public SwerveDriveCommonConfig setPeriod(Time period) {
        this.period = period;
        return this;
    }

    /**
     * Sets the odometry standard deviation vector. Default is [.05, .05,
     * Units.degreesToRadians(5)]
     * 
     * @param std odometry standard deviation vector
     * @return current configuration object
     */
    public SwerveDriveCommonConfig setStateStd(Vector<N3> std) {
        this.stateStd = std;
        return this;
    }

    /**
     * Sets the default vision standard deviation vector. Default is [0.5, 0.5,
     * Units.degreesToRadians(30)]
     * 
     * @param std default vision standard deviation vector
     * @return current configuration object
     */
    public SwerveDriveCommonConfig setVisionStd(Vector<N3> std) {
        this.visionStd = std;
        return this;
    }

    /**
     * Sets the linear position display units. Defaults to Meters.
     * @param unit the linear position display units
     * @return current configuration object
     */
    public SwerveDriveCommonConfig setLinPosUnits(DistanceUnit unit) {
        linPosUnit = unit;
        return this;
    }


    /**
     * Sets the linear time display units. Defaults to Seconds.
     * @param unit the linear time display units
     * @return current configuration object
     */
    public SwerveDriveCommonConfig setLinTimeUnits(TimeUnit unit) {
        linTimeUnit = unit;
        return this;
    }

    /**
     * Sets the angular position display units. Defaults to Meters.
     * @param unit the angular position display units
     * @return current configuration object
     */
    public SwerveDriveCommonConfig setAngPosUnits(AngleUnit unit) {
        angPosUnit = unit;
        return this;
    }


    /**
     * Sets the angular time display units. Defaults to Seconds.
     * @param unit the angular time display units
     * @return current configuration object
     */
    public SwerveDriveCommonConfig setAngTimeUnits(TimeUnit unit) {
        angTimeUnit = unit;
        return this;
    }

    /**
     * Copies another config into the current config
     * 
     * @param other other config to copy
     * @return current configuration object
     */
    public SwerveDriveCommonConfig copyConfig(SwerveDriveCommonConfig other) {
        this.linearCtrlConfig = other.linearCtrlConfig;
        this.angularCtrlConfig = other.angularCtrlConfig;

        this.linearPPPID = other.linearPPPID;
        this.angularPPPID = other.angularPPPID;
        this.period = other.period;
        this.stateStd = other.stateStd;
        this.visionStd = other.visionStd;

        return this;
    }
}
