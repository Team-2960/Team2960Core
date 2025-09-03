package frc.lib2960.config.subsystem;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import frc.lib2960.config.controller.AngularControllerConfig;
import frc.lib2960.config.controller.LinearControllerConfig;
import frc.lib2960.config.controller.PIDConfig;

public class SwerveDriveBaseConfig {
    /** Config for linear control of the robot base. Defaults to a default LinearControlConfig. */
    public LinearControllerConfig linearCtrlConfig = new LinearControllerConfig();
    /** Config for anguler control of the robot base. Defaults to a default AngularControlConfig. */
    public AngularControllerConfig angleCtrlConfig = new AngularControllerConfig();

    /** PID Config for linear PathPlanner control. Defaults to kP, kI, & kD set to zero. */
    public PIDConfig linearPPPID = new PIDConfig(0, 0, 0);
    /** PID Config for angular PathPlanner control. Defaults to kP, kI, & kD set to zero. */
    public PIDConfig angularPPPID = new PIDConfig(0, 0, 0);
    /** Update period of the drive base */
    public Time period = Seconds.of(0.02);
    /** Odometry standard deviation vector. Defaults to [.05, .05, Units.degreesToRadians(5)] */
    public Vector<N3> stateStd = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    /** Default vision standard deviation vector. Defaults to [.5, .5, Units.degreesToRadians(30)] */
    public Vector<N3> visionStd = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30));

    /**
     * Set the linear PathPlanner PID configuration. Default is kP, kI, and kD are
     * all set to zero,
     * 
     * @param pidConfig linear PathPlanner PID
     * @return current configuration object
     */
    public SwerveDriveBaseConfig setLinearPPPID(PIDConfig pidConfig) {
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
    public SwerveDriveBaseConfig setAngularPPPID(PIDConfig pidConfig) {
        this.angularPPPID = pidConfig;
        return this;
    }

    /**
     * Sets the update period. Default is set to 20 ms.
     * 
     * @param period update period
     * @return current configuration object
     */
    public SwerveDriveBaseConfig setPeriod(Time period) {
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
    public SwerveDriveBaseConfig setStateStd(Vector<N3> std) {
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
    public SwerveDriveBaseConfig setVisionStd(Vector<N3> std) {
        this.visionStd = std;
        return this;
    }

    /**
     * Copies another config into the current config
     * @param other other config to copy
     * @return current configuration object
     */
    public SwerveDriveBaseConfig copyConfig(SwerveDriveBaseConfig other) {
        this.linearCtrlConfig = other.linearCtrlConfig;
        this.angleCtrlConfig = other.angleCtrlConfig;
        
        this.linearPPPID = other.linearPPPID;
        this.angularPPPID = other.angularPPPID;
        this.period = other.period;
        this.stateStd = other.stateStd;
        this.visionStd = other.visionStd;

        return this;
    }
}
