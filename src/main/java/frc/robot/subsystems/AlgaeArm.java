package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import frc.lib2960.config.device.MotorConfig;
import frc.lib2960.config.subsystem.AngularMotorMechConfig;
import frc.lib2960.subsystem.motor.AngularMotorMech;

public class AlgaeArm extends AngularMotorMech {
    private final SparkMax motor;

    private final AbsoluteEncoder absEncoder;
    private final RelativeEncoder encoder;

    /**
     * Constructor
     * 
     * @param config Angular motor mechanism configuration
     */
    public AlgaeArm(AngularMotorMechConfig config, MotorConfig motorConfig) {
        super(config);

        // Create motor controller
        motor = new SparkMax(motorConfig.id, MotorType.kBrushless);

        // Configure motor controller
        SparkMaxConfig flexConfig = new SparkMaxConfig();
        flexConfig.inverted(motorConfig.invert);
        flexConfig.alternateEncoder.velocityConversionFactor(1 / 60);

        // TODO Add motor controller based soft limits

        motor.configure(flexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // Get encoders
        absEncoder = motor.getAbsoluteEncoder();
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
        result.mut_replace(absEncoder.getPosition(), Rotations);
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

}
