package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import frc.lib2960.config.subsystem.LinearMotorMechConfig;
import frc.lib2960.subsystem.motor.LinearMotorMech;

public class CoralRoller extends LinearMotorMech {
    private SparkFlex[] motors;

    private RelativeEncoder encoder = null;

    /**
     * Constructor
     * 
     * @param config Linear motor mechanism configuration
     */
    public CoralRoller(LinearMotorMechConfig config) {
        super(config);

        // Create the motors
        motors = new SparkFlex[config.common.motorConfigs.length];
        for (int i = 0; i < motors.length; i++) {
            var motorConfig = config.common.motorConfigs[i];

            motors[i] = new SparkFlex(motorConfig.id, MotorType.kBrushless);

            var flexConfig = new SparkFlexConfig();
            flexConfig.inverted(motorConfig.invert);

            motors[i].configure(flexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }

        // FInd the motor controller with the same ID as the encoder ID
        if (config.common.encoderConfig.isPresent()) {
            var encoderConfig = config.common.encoderConfig.get();
            for (int i = 0; i < config.common.motorConfigs.length; i++) {
                if (encoderConfig.id == config.common.motorConfigs[i].id) {
                    encoder = motors[i].getEncoder();
                    var flexConfig = new SparkFlexConfig();

                    double posConv = config.pulleyCircumfrance.in(Meters) * encoderConfig.gearRatio;

                    flexConfig.encoder.inverted(encoderConfig.invert)
                       .positionConversionFactor(posConv)
                       .velocityConversionFactor(posConv / 60);

                    break;
                }
            }

            // Throw exception if no encoder is found
            if (encoder == null)
                throw new IllegalArgumentException("Encoder ID not set to the same ID as a motor is this mechanism");
        }
    }

    /**
     * Sets the output voltage for all the motors
     * 
     * @param volts output voltage for all the motors
     */
    @Override
    public void setMotorVoltage(Voltage volts) {
        for (var motor : motors)
            motor.setVoltage(volts);
    }

    /**
     * Gets the current position of the mechanism.
     * 
     * @param result mutable object to store the result
     */
    @Override
    public void getPosition(MutDistance result) {
        if (encoder != null) {
            result.mut_replace(encoder.getPosition(), Meters);
        } else {
            result.mut_replace(0, Meters);
        }
    }

    /**
     * Resets the position of the encoder.
     * @param position  new position of the encoder.
     */
    public void resetPosition(Distance position) {
        if (encoder != null) {
            encoder.setPosition(position.in(Meters));
        } else {
            // TODO Improve warning
            System.out.println("Encoder not set for CoralRoller. Encoder not reset");
        }
    }

    /**
     * Gets the current velocity of the mechanism.
     * 
     * @param result mutable object to store the result
     */
    @Override
    public void getVelocity(MutLinearVelocity result) {
        if (encoder != null) {
            result.mut_replace(encoder.getVelocity(), MetersPerSecond);
        } else {
            result.mut_replace(0, MetersPerSecond);
        }
    }

    /**
     * Gets the current voltage applied to each motor on the mechanism
     * 
     * @param result mutable object to store the result
     */
    @Override
    public void getVoltage(MutVoltage result) {
        if (motors.length > 1) {
            result.mut_replace(motors[0].getAppliedOutput() * motors[0].getBusVoltage(), Volts);
        } else {
            result.mut_replace(0, Volts);
        }
    }

}
