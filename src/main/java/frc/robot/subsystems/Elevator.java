package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib2960.config.device.MotorConfig;
import frc.lib2960.config.subsystem.LinearMotorMechConfig;
import frc.lib2960.subsystem.motor.LinearMotorMech;

public class Elevator extends LinearMotorMech {
    private final SparkFlex motor;

    private final RelativeEncoder encoder;
    @SuppressWarnings("unused")
    private final SparkLimitSwitch forwardLimit;
    private final SparkLimitSwitch reverseLimit;

    private final Trigger posResetTrigger;

    /**
     * Constructor
     * 
     * @param config Linear motor mechanism configuration
     */
    public Elevator(LinearMotorMechConfig config, MotorConfig motorConfig) {
        super(config);

        // Create motor controller
        motor = new SparkFlex(motorConfig.id, MotorType.kBrushless);

        // Configure motor controller
        double posConv = config.pulleyCircumfrance.in(Meters) * motorConfig.gearRatio;

        SparkFlexConfig flexConfig = new SparkFlexConfig();
        flexConfig.inverted(motorConfig.invert);
        flexConfig.encoder.positionConversionFactor(posConv);
        flexConfig.encoder.velocityConversionFactor(posConv / 60);

        // TODO Add motor controller based soft limits

        motor.configure(flexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // Get encoders
        encoder = motor.getEncoder();

        // Get Sensors
        forwardLimit = motor.getForwardLimitSwitch();
        reverseLimit = motor.getReverseLimitSwitch();

        // Setup Triggers
        posResetTrigger = new Trigger(reverseLimit::isPressed);
        posResetTrigger.onTrue(Commands.runOnce(()->encoder.setPosition(0)));

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
    public void getPosition(MutDistance result) {
        result.mut_replace(encoder.getPosition(), Meters);
    }

    /**
     * Gets the current position of the mechanism.
     * 
     * @return current position of the mechanism.
     */
    public Distance getPosition() {
        return Meters.of(encoder.getPosition());
    }

    /**
     * Resets the position of the encoder.
     * 
     * @param position new position of the encoder.
     */
    public void resetPosition(Distance position) {
        encoder.setPosition(position.in(Meters));
    }

    /**
     * Gets the current velocity of the mechanism.
     * 
     * @param result mutable object to store the result
     */
    @Override
    public void getVelocity(MutLinearVelocity result) {
        result.mut_replace(encoder.getVelocity(), MetersPerSecond);
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
