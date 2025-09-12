package frc.lib2960.config.controller;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class FFConfig {

    // TODO Implement using Units library
    public double kS;
    public double kG;
    public double kV;
    public double kA;

    /**
     * Constructor
     * 
     * @param kS Voltage to overcome static friction
     * @param kG Voltage to overcome gravity
     * @param kV Voltage per angular velocity of the system
     * @param kA Voltage per angular acceleration of the system
     */
    public FFConfig(double kS, double kG, double kV, double kA) {
        this.kS = kS;
        this.kG = kG;
        this.kV = kV;
        this.kA = kA;
    }

    /**
     * Constructor
     * 
     * @param kS Voltage to overcome static friction
     * @param kG Voltage to overcome gravity
     * @param kV Voltage per angular velocity of the system
     */
    public FFConfig(double kS, double kG, double kV) {
        this(kS, kG, kV, 0);
    }

    /**
     * Constructor
     * 
     * @param kS Voltage to overcome static friction
     * @param kV Voltage per angular velocity of the system
     */
    public FFConfig(double kS, double kV) {
        this(kS, 0, kV, 0);
    }

    /**
     * Generates a new simple motor feed forward controller
     * 
     * @return new simple motor feed forward controller
     */
    public SimpleMotorFeedforward getSimpleMotorFF() {
        return new SimpleMotorFeedforward(kS, kV, kA);
    }

    /**
     * Generates a new arm feed forward controller
     * 
     * @return new arm feed forward controller
     */
    public ArmFeedforward getArmFF() {
        return new ArmFeedforward(kS, kG, kV, kA);
    }

    /**
     * Generates a new elevator feed forward controller
     * 
     * @return new elevator feed forward controller
     */
    public ElevatorFeedforward getElevatorFF() {
        return new ElevatorFeedforward(kS, kG, kV, kA);
    }

}
