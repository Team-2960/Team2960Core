package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2960.subsystem.motor.AngularMotorMech;

public class AlgaeRoller extends AngularMotorMech {
    private final AlgaeRollerConfig config;

    private final SparkMax motor;

    private final RelativeEncoder encoder;

    /**
     * Constructor
     * 
     * @param config Angular motor mechanism configuration
     */
    public AlgaeRoller(AlgaeRollerConfig config) {
        super(config.motorMechConfig);
        this.config = config;

        // Create motor controller
        motor = new SparkMax(config.motorConfig.id, MotorType.kBrushless);

        // Configure motor controller
        SparkMaxConfig flexConfig = new SparkMaxConfig();
        flexConfig.inverted(config.motorConfig.invert);
        flexConfig.alternateEncoder.velocityConversionFactor(1 / 60);

        motor.configure(flexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // Get encoders
        encoder = motor.getAlternateEncoder();

        // Set Default Command 
        setDefaultCommand(getHoldPosCmd());
    }

    /**
     * Sets the output voltage for all the motors
     * 
     * @param volts output voltage for all the motors
     */
    @Override
    public void setMotorVoltage(Voltage volts) {
        motor.setVoltage(volts);
    }

    /**
     * Gets the current position of the mechanism.
     * 
     * @param result mutable object to store the result
     */
    @Override
    public void getPosition(MutAngle result) {
        result.mut_replace(encoder.getPosition(), Rotations);
    }

    /**
     * Gets the current velocity of the mechanism.
     * 
     * @param result mutable object to store the result
     */
    @Override
    public void getVelocity(MutAngularVelocity result) {
        result.mut_replace(encoder.getVelocity(), RotationsPerSecond);
    }

    /**
     * Gets the current voltage applied to each motor on the mechanism
     * 
     * @param result mutable object to store the result
     */
    @Override
    public void getVoltage(MutVoltage result) {
        result.mut_replace(motor.getAppliedOutput() * motor.getBusVoltage(), Volts);
    }

    /**
     * 
     * @param angle
     */
    public void resetPosition(Angle angle) {
        encoder.setPosition(angle.in(Rotations));
    }

    /*********************/
    /* Command Factories */
    /*********************/
    /**
     * Gets a new command to eject algae from the gripper
     * 
     * @return new command to eject algae from the gripper
     */
    public Command getEjectCmd() {
        return this.runEnd(
                () -> setVoltage(config.ejectVolt),
                () -> setVoltage(Volts.zero()));
    }

    /**
     * Gets a new command to intake algae into the gripper
     * 
     * @return new command to intake algae into the gripper
     */
    public Command getIntakeCmd() {
        return this.runEnd(
            () -> setVoltage(config.intakeVolt),
            () -> setVoltage(Volts.zero()));
    }

}
