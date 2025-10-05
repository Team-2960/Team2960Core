package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import com.revrobotics.spark.SparkLimitSwitch;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib2960.config.device.MotorConfig;
import frc.lib2960.subsystem.motor.LinearSparkFlexMech;
import frc.lib2960.subsystem.motor.LinearSparkMechConfig;

public class CoralRoller extends LinearSparkFlexMech {
    private final DigitalInput intakeSensor;
    private final SparkLimitSwitch grippedSensor;

    /**
     * Constructor
     * 
     * @param config Linear motor mechanism configuration
     */
    public CoralRoller(LinearSparkMechConfig config, int intakeSensorID, MotorConfig motorConfig) {
        super(config, motorConfig);

        // Get Sensors
        intakeSensor = new DigitalInput(intakeSensorID);
        grippedSensor = motor.getForwardLimitSwitch();

        // Set Default Command
        setDefaultCommand(getHoldPosCmd());

        tab.add("Coral Sensor Status", getCoralRollerStatusSendable());

    }

    /**
     * Gets sendable status for the Coral roller
     * 
     * @return sendable status for the Coral roller
     */
    public Sendable getCoralRollerStatusSendable() {
        return new Sendable() {

            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addBooleanProperty("Intake Sensor", () -> intakeSensor.get(), null);
                builder.addBooleanProperty("Gripper Sensor", () -> grippedSensor.isPressed(), null);
            }

        };
    }

    /**
     * Creates trigger for the intake sensor
     * 
     * @return trigger for the intake sensor
     */
    public Trigger getIntakeTrigger() {
        return new Trigger(this::coralAtIntake);
    }

    /**
     * Creates trigger for the gripper sensor
     * 
     * @return trigger for the gripper sensor
     */
    public Trigger getGripperTrigger() {
        return new Trigger(this::coralInGripper);
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
     * Gets a new command to reset the mechanism position
     * 
     * @param distance distance to set to the mechanism
     * @return new command to reset the mechanism position
     */
    public Command getResetPosCmd(Distance distance) {
        return Commands.runOnce(() -> resetPosition(distance));
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
