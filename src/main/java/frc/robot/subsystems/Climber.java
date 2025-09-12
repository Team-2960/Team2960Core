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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib2960.subsystem.motor.AngularMotorMech;

public class Climber extends AngularMotorMech {
    private final ClimberConfig config;

    private final SparkFlex motor;

    private final AbsoluteEncoder absEncoder;
    private final RelativeEncoder encoder;

    /**
     * Constructor
     * @param config    Climber configuration
     */
    public Climber(ClimberConfig config) {
        super(config.motorMechConfig);
        this.config = config;

        // Create motor controller
        motor = new SparkFlex(config.motorConfig.id, MotorType.kBrushless);

        // Configure motor controller
        SparkFlexConfig flexConfig = new SparkFlexConfig();
        flexConfig.inverted(config.motorConfig.invert);
        flexConfig.externalEncoder.velocityConversionFactor(1 / 60);

        if (config.motorMechConfig.controlConfig.minimum.isPresent()) {
            flexConfig.softLimit.reverseSoftLimit(config.motorMechConfig.controlConfig.minimum.get().in(Rotations));
            flexConfig.softLimit.reverseSoftLimitEnabled(true);
        }

        if (config.motorMechConfig.controlConfig.maximum.isPresent()) {
            flexConfig.softLimit.forwardSoftLimit(config.motorMechConfig.controlConfig.minimum.get().in(Rotations));
            flexConfig.softLimit.forwardSoftLimitEnabled(true);
        }

        motor.configure(flexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // Get encoders
        absEncoder = motor.getAbsoluteEncoder();
        encoder = motor.getExternalEncoder();

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

    /*********************/
    /* Command Factories */
    /*********************/
    /**
     * Get a new command to extend the climber
     * 
     * @return new command to extend the climber
     */
    public Command getExtendCmd() {
        return this.runEnd(
                () -> setVoltage(config.extendVolt),
                () -> setVoltage(Volts.zero()));
    }

    /**
     * Get a new command to retract the climber
     * 
     * @return new command to extend the climber
     */
    public Command getRetractCmd() {
        return this.runEnd(
                () -> setVoltage(config.retractVolt),
                () -> setVoltage(Volts.zero()));
    }

    /**
     * Get a new command to extend the climber until its at or beyond its extended
     * position
     * 
     * @return new command to extend the climber until its at or beyond its extended
     *         position
     */
    public Command getAutoExtendCmd() {
        return Commands.deadline(
                getWaitUntilExtend(),
                getExtendCmd());
    }

    /**
     * Get a new command to retract the climber until its at or beyond its climb
     * position
     * 
     * @return new command to retract the climber until its at or beyond its climb
     *         position
     */
    public Command getAutoClimbCmd() {
        return Commands.deadline(
                getWaitUntilClimb(),
                getRetractCmd());
    }

    /**
     * Get a new command to retract the climber until its at or beyond its home
     * position
     * 
     * @return new command to retract the climber until its at or beyond its home
     *         position
     */
    public Command getAutoHomeCmd() {
        return Commands.deadline(
                getWaitUntilHome(),
                getRetractCmd());
    }

    /**
     * Get a new command to wait until the climber is at or beyond the extend
     * position
     * 
     * @return new command to wait until the climber is at or beyond the extend
     *         position
     */
    public Command getWaitUntilExtend() {
        return Commands.waitUntil(() -> {
            MutAngle result = Rotations.mutable(0);
            getPosition(result);
            return result.gte(config.extendAngle);
        });
    }

    /**
     * Get a new command to wait until the climber is at or beyond the climb
     * position
     * 
     * @return new command to wait until the climber is at or beyond the climb
     *         position
     */
    public Command getWaitUntilClimb() {
        return Commands.waitUntil(() -> {
            MutAngle result = Rotations.mutable(0);
            getPosition(result);
            return result.lte(config.climbAngle);
        });
    }

    /**
     * Get a new command to wait until the climber is at or beyond the home position
     * 
     * @return new command to wait until the climber is at or beyond the home
     *         position
     */
    public Command getWaitUntilHome() {
        return Commands.waitUntil(() -> {
            MutAngle result = Rotations.mutable(0);
            getPosition(result);
            return result.lte(config.homeAngle);
        });
    }
}
