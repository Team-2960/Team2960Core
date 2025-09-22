package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ArmElevControl {
    private final CoralArm arm;
    private final Elevator elevator;
    private final Angle armTol;
    private final Distance elevTol;

    /**
     * Constructor
     * @param arm       Coral Arm Subsystem
     * @param elevator  Elevator subsystem
     */
    public ArmElevControl(CoralArm arm, Elevator elevator, Angle armTol, Distance elevTol) {
        this.arm = arm;
        this.elevator = elevator;
        this.armTol = armTol;
        this.elevTol = elevTol;
    }

    /*********************/
    /* Command Factories */
    /*********************/
    /**
     * Gets a new command sequence to move the arm and elevator to the intake position
     * @return new command sequence to move the arm and elevator to the intake position
     */
    public Command getGotoIntakeCmd() {
        return Commands.sequence(
            getPrepareTravelCmd(),
            getElevMoveCmd("Intake"),
            getEndTravelCmd("Intake", "Intake")
        );
    }

    /**
     * Gets a new command sequence to move the arm and elevator to the L1 position
     * @return new command sequence to move the arm and elevator to the L1 position
     */
    public Command getGotoL1() {
        return Commands.sequence(
            getPrepareTravelCmd(),
            getElevMoveCmd("L1"),
            getEndTravelCmd("L1", "L1")
        );
    }

    /**
     * Gets a new command sequence to move the arm and elevator to the L2 position
     * @return new command sequence to move the arm and elevator to the L2 position
     */
    public Command getGotoL2() {
        return Commands.sequence(
            getPrepareTravelCmd(),
            getElevMoveCmd("L2"),
            getEndTravelCmd("L2", "L2")
        );
    }

    /**
     * Gets a new command sequence to move the arm and elevator to the L3 position
     * @return new command sequence to move the arm and elevator to the L3 position
     */
    public Command getGotoL3() {
        return Commands.sequence(
            getPrepareTravelCmd(),
            getElevMoveCmd("L3"),
            getEndTravelCmd("L3", "L3")
        );
    }

    /**
     * Gets a new command sequence to move the arm and elevator to the L4 position
     * @return new command sequence to move the arm and elevator to the L4 position
     */
    public Command getGotoL4() {
        return Commands.sequence(
            getPrepareTravelCmd(),
            getElevMoveCmd("L4"),
            getEndTravelCmd("L4", "L4")
        );
    }

    /**
     * Gets a new command sequence to move the arm and elevator to the low algae position
     * @return new command sequence to move the arm and elevator to the low algae position
     */
    public Command getGotoLowAlgae() {
        return Commands.sequence(
            getPrepareTravelCmd(),
            getElevMoveCmd("Low Algae"),
            getEndTravelCmd("Remove Algae", "Low Algae")
        );
    }

    /**
     * Gets a new command sequence to move the arm and elevator to the high algae position
     * @return new command sequence to move the arm and elevator to the high algae position
     */
    public Command getGotoHighAlgae() {
        return Commands.sequence(
            getPrepareTravelCmd(),
            getElevMoveCmd("High Algae"),
            getEndTravelCmd("Remove Algae", "High Algae")
        );
    }

    /**
     * Gets a new command to Move to pre-travel position
     * @return  new command to Move to pre-travel position
     */
    public Command getPrepareTravelCmd() {
        return Commands.race(
            arm.getPosPresetCmd("Travel", armTol),
            elevator.getHoldPosCmd()
        );
    }

    /**
     * Gets a new command to move elevator to target
     * @param elevPreset name elevator preset to move to
     * @return new command to move elevator to target
     */
    public Command getElevMoveCmd(String elevPreset) {
        return Commands.race(
            arm.getPosPresetCmd("Travel"),
            elevator.getPosPresetCmd(elevPreset, elevTol)
        );
    }
    
    /**
     * Gets a new command to end goto sequence
     * @param armPreset name arm preset to move to
     * @param elevPreset name elevator preset to move to
     * @return
     */
    public Command getEndTravelCmd(String armPreset, String elevPreset) {
        return Commands.race(
            arm.getPosPresetCmd(armPreset, armTol), 
            elevator.getPosPresetCmd(elevPreset)
        );
    }
}
