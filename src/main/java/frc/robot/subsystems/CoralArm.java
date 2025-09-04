package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import frc.lib2960.config.subsystem.AngularMotorMechConfig;
import frc.lib2960.subsystem.motor.AngularMotorMech;

public class CoralArm extends AngularMotorMech {
    private SparkFlex[] motors;

    private AbsoluteEncoder absEncoder = null;
    private RelativeEncoder encoder = null;
    
    public CoralArm(AngularMotorMechConfig config) {
        super(config);

        // Create the motors
        motors = new SparkFlex[config.common.motorConfigs.length];
        for (int i = 0; i < motors.length; i++) {
            var motorConfig = config.common.motorConfigs[i];

            motors[i] = new SparkFlex(motorConfig.id, MotorType.kBrushless);

            var flexConfig = new SparkFlexConfig();
            flexConfig.inverted(motorConfig.invert);
            flexConfig.externalEncoder.velocityConversionFactor(1 / 60);

            motors[i].configure(flexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }

        // FInd the motor controller with the same ID as the encoder ID
        if (config.common.encoderConfig.isPresent()) {
            var encoderConfig = config.common.encoderConfig.get();
            for (int i = 0; i < config.common.motorConfigs.length; i++) {
                if (encoderConfig.id == config.common.motorConfigs[i].id) {
                    absEncoder = motors[i].getAbsoluteEncoder();
                    encoder = motors[i].getExternalEncoder();
                    break;
                }
            }

            // Throw exception if no encoder is found
            if (absEncoder == null)
                throw new IllegalArgumentException("Encoder ID not set to the same ID as a motor is this mechamism");
        }
    }

    @Override
    public void setMotorVoltage(Voltage volts) {
        for (var motor : motors)
            motor.setVoltage(volts);
    }

    @Override
    public void getPosition(MutAngle result) {
        if (absEncoder != null) {
            result.mut_replace(absEncoder.getPosition(), Rotations);
        } else {
            result.mut_replace(0, Rotations);
        }
    }

    @Override
    public void getVelocity(MutAngularVelocity result) {
        if (encoder != null) {
            result.mut_replace(encoder.getVelocity(), RotationsPerSecond);
        } else {
            result.mut_replace(0, RotationsPerSecond);
        }
    }

    @Override
    public void getVoltage(MutVoltage result) {
        if(motors.length > 1) {
            result.mut_replace(motors[0].getAppliedOutput() * motors[0].getBusVoltage(), Volts);
        }else {
            result.mut_replace(0, Volts);
        }
    }

}
