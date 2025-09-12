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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib2960.subsystem.motor.LinearMotorMech;

public class CoralRoller extends LinearMotorMech {
    private final CoralRollerConfig config;

    private final SparkFlex motor;

    private final RelativeEncoder encoder;
    private final DigitalInput intakeSensor;
    private final SparkLimitSwitch grippedSensor;

    private final Trigger intakeTrigger;

    /**
     * Constructor
     * 
     * @param config Linear motor mechanism configuration
     */
    public CoralRoller(CoralRollerConfig config) {
        super(config.motorMechConfig);
        this.config = config;

        // Create motor controller
        motor = new SparkFlex(config.motorConfig.id, MotorType.kBrushless);

        // Configure motor controller
        SparkFlexConfig flexConfig = new SparkFlexConfig();
        flexConfig.inverted(config.motorConfig.invert);
        flexConfig.externalEncoder.velocityConversionFactor(1 / 60);

        motor.configure(flexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // Get encoders
        encoder = motor.getEncoder();

        // Get Sensors
        intakeSensor = new DigitalInput(config.intakeSensorID);
        grippedSensor = motor.getForwardLimitSwitch();

        // Setup Triggers
        intakeTrigger = new Trigger(this::coralAtIntake);
        intakeTrigger.onTrue(getAutoIntakeCmd(Meters.zero()));

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

    /**
     * Checks if a coral is at the intake sensor
     * 
     * @return true if a coral is at the intake, false otherwise
     */
    public boolean coralAtIntake() {
        return intakeSensor.get();
    }

    /**
     * Checks if a coral is in the gripper
     * 
     * @return true if a coral is in the gripper, false otherwise
     */
    public boolean coralInGripper() {
        return grippedSensor.isPressed();
    }

    /*********************/
    /* Command Factories */
    /*********************/

    /**
     * Gets a new command to eject coral from the gripper
     * 
     * @return new command to eject coral from the gripper
     */
    public Command getEjectCmd() {
        return this.runEnd(
                () -> setVoltage(config.ejectVolt),
                () -> setVoltage(Volts.zero()));
    }

    /**
     * Ejects coral from the gripper until a specific time has elapsed
     * 
     * @param distance distance to run after coral is no longer in the gripper
     * @return new command to eject coral from the gripper until a specific time has
     *         elapsed
     */
    public Command getAutoEjectCmd(Distance distance) {
        return Commands.deadline(
                getWaitForCoralInGripperCmd(distance),
                getEjectCmd());
    }

    /**
     * Gets a new command to intake coral into the gripper
     * 
     * @return new command to intake coral into the gripper
     */
    public Command getIntakeCmd() {
        return this.runEnd(
            () -> setVoltage(config.intakeVolt),
            () -> setVoltage(Volts.zero()));
    }

    /**
     * Gets a new command to intake coral into the gripper until no coral is present
     * at the intake
     * 
     * @return new command to intake coral into the gripper until no coral is
     *         present at the intake
     */
    public Command getAutoIntakeCmd(Distance distance) {
        return Commands.deadline(
                getWaitForCoralNotAtIntakeCmd(distance),
                getIntakeCmd());
    }

    /**
     * Gets a new command to reverse the coral gripper
     * 
     * @return new command to reverse the coral gripper
     */
    public Command getReverseCommand() {
        return this.runEnd(
            () -> setVoltage(config.reverseVolt),
            () -> setVoltage(Volts.zero()));
    }

    /**
     * Gets a new command to reset the mechanism position
     * 
     * @param distance distance to set to the mechanism
     * @return new command to reset the mechanism position
     */
    public Command getResetPosCmd(Distance distance) {
        return this.runOnce(() -> resetPosition(distance));
    }

    /**
     * Gets a new command that waits until a coral is at the gripper intake
     * 
     * @return new command that waits until a coral is at the gripper intake
     */
    public Command getWaitForCoralAtIntakeCmd(Distance distance) {
        return Commands.sequence(
                Commands.waitUntil(this::coralAtIntake),
                getWaitUntilDistCmd(distance));
    }

    /**
     * Gets a new command that waits until a coral is at the gripper intake
     * 
     * @return new command that waits until a coral is at the gripper intake
     */
    public Command getWaitForCoralNotAtIntakeCmd(Distance distance) {
        return Commands.sequence(
                Commands.waitUntil(() -> this.coralAtIntake()),
                getWaitUntilDistCmd(distance));
    }

    /**
     * Gets a new command that waits until a coral is in the gripper
     * 
     * @param distance distance to run after a coral is detected in the gripper
     * @return new command that waits until a coral is in the gripper
     */
    public Command getWaitForCoralInGripperCmd(Distance distance) {
        return Commands.sequence(
                Commands.waitUntil(this::coralInGripper),
                getWaitUntilDistCmd(distance));
    }

    /**
     * Gets a new command that waits until a coral is not in the gripper
     * 
     * @param distance distance to run after a coral is no longer detected in the
     *                 gripper
     * @return new command that waits until a coral is not in the gripper
     */
    public Command getWaitForCoralNotInGripperCmd(Distance distance) {
        return Commands.sequence(
                Commands.waitUntil(() -> !this.coralInGripper()),
                getWaitUntilDistCmd(distance));
    }

    /**
     * Gets a new command that waits until the gripper has reached a given distance
     * 
     * @param distance distance to travel
     * @return new command that waits until the gripper has reached a given distance
     */
    public Command getWaitUntilDistCmd(Distance distance) {
        return Commands.sequence(
                getResetPosCmd(Meters.zero()),
                Commands.waitUntil(() -> {
                    MutDistance result = Meters.mutable(0);
                    getPosition(result);
                    return result.gte(distance);
                }));
    }
}
