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

    public final LinearControllerConfig linearCtrlConfig = new LinearControllerConfig();
    public final AngularControllerConfig angleCtrlConfig = new AngularControllerConfig();

    private String name = "Drivetrain";
    private PIDConfig linearPPPID = new PIDConfig(0, 0, 0);
    private PIDConfig angularPPPID = new PIDConfig(0, 0, 0);
    private Time period = Seconds.of(0.02);
    private Vector<N3> stateStd = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    private Vector<N3> visionStd = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30));

    /**
     * Set the configured name. Default is "Drivetrain".
     * 
     * @param name set the configured name
     * @return current configuration object
     */
    public SwerveDriveBaseConfig setName(String name) {
        this.name = name;
        return this;
    }

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
     * Gets the configured name
     * 
     * @return configured name
     */
    public String getName() {
        return name;
    }

    /**
     * Gets the linear PathPlanner PID configuration
     * 
     * @return linear PathPlanner PID configuration
     */
    public PIDConfig getLinearPPPIDConfig() {
        return linearPPPID;
    }

    /**
     * Gets the angular PathPlanner PID configuration
     * 
     * @return angular PathPlanner PID configuration
     */
    public PIDConfig getAngularPPPIDConfig() {
        return angularPPPID;
    }

    /**
     * Gets the update period
     * 
     * @return update period
     */
    public Time getPeriod() {
        return period;
    }

    /**
     * Gets the odometry standard deviation vector
     * @return odometry standard deviation vector
     */
    public Vector<N3> getStateStd() {
        return stateStd;
    }


    /**
     * Gets the default vision standard deviation vector
     * @return default vision standard deviation vector
     */
    public Vector<N3> getVisionStd() {
        return visionStd;
    }
}
