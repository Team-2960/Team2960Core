
package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2960.helper.Elastic;
import frc.lib2960.helper.PathPlanner;
import frc.lib2960.subsystem.drivetrain.swerve.RevFlexMaxSwerveModule;
import frc.lib2960.subsystem.vision.AprilTagPipeline;
import frc.robot.FieldLayout.ReefBranchOffset;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.AlgaeRoller;
import frc.robot.subsystems.ArmElevControl;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.CoralRoller;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

/**
 * Class to contain all the robot subsystems and items that are not specific to
 * a single or group of subsystems
 */
public class RobotContainer {

    // Subsystems
    private final RevFlexMaxSwerveModule[] swerveModules = {
            new RevFlexMaxSwerveModule(Constants.swerveModuleCommonConfig, Constants.lfConfig),
            new RevFlexMaxSwerveModule(Constants.swerveModuleCommonConfig, Constants.rfConfig),
            new RevFlexMaxSwerveModule(Constants.swerveModuleCommonConfig, Constants.lrConfig),
            new RevFlexMaxSwerveModule(Constants.swerveModuleCommonConfig, Constants.rrConfig),
    };

    private final Drivetrain drivetrain = new Drivetrain(Constants.swerveDriveConfig,
            swerveModules);
    private final CoralArm coralArm = new CoralArm(Constants.coralArmConfig, Constants.coralArmMotorConfig);
    private final CoralRoller coralRoller = new CoralRoller(Constants.coralRollerConfig);
    private final Elevator elevator = new Elevator(Constants.elevatorConfig, Constants.elevatorMotorConfig);
    private final ArmElevControl armElevCtrl = new ArmElevControl(coralArm, elevator,
            Constants.coralArmPosTol,
            Constants.elevatorPosTol);
    private final AlgaeArm algaeArm = new AlgaeArm(Constants.algaeArmConfig, Constants.algaeArmMotorConfig);
    private final AlgaeRoller algaeRoller = new AlgaeRoller(Constants.algaeRollerConfig);
    private final Climber climber = new Climber(Constants.climberConfig);

    /*
    @SuppressWarnings("unused")
    private final AprilTagPipeline leftCamera = new AprilTagPipeline(Constants.leftCameraConfig, drivetrain);
    @SuppressWarnings("unused")
    private final AprilTagPipeline frontCamera = new AprilTagPipeline(Constants.frontCameraConfig, drivetrain);
    @SuppressWarnings("unused")
    private final AprilTagPipeline rightCamera = new AprilTagPipeline(Constants.rightCameraConfig, drivetrain);
    */
    // Joysticks
    private final CommandXboxController driveCtrl = new CommandXboxController(Constants.driverCtrlID);
    private final CommandXboxController opCtrl = new CommandXboxController(Constants.opCtrlID);

    // Helper Units
    private final MutLinearVelocity xTeleopVel = MetersPerSecond.mutable(0);
    private final MutLinearVelocity yTeleopVel = MetersPerSecond.mutable(0);
    private final MutAngularVelocity rTeleopVel = DegreesPerSecond.mutable(0);

    /**
     * Constructor
     */
    public RobotContainer() {
        // Initialize Elastic Layout server
        Elastic.startLayoutServer();
        
        // Init Driver Controls
        initDrivetrainCtrl();
        initCoralCtrl();
        initAlgaeCtrl();
        initClimberCtrl();



        // Init PathPlanner
        initNamedCommands();
        initPathPlanner();
    }

    private final void initDrivetrainCtrl() {
        // Init Teleop Drivetrain Controls
        drivetrain.setDefaultCommand(drivetrain.getVelocityControlCmd(
                () -> xTeleopVel.mut_replace(
                        MathUtil.applyDeadband(-driveCtrl.getLeftX(), Constants.linDriveDB)
                                * Constants.driveLinMaxVel.in(MetersPerSecond),
                        MetersPerSecond),
                () -> yTeleopVel.mut_replace(
                        MathUtil.applyDeadband(-driveCtrl.getLeftY(), Constants.linDriveDB)
                                * Constants.driveLinMaxVel.in(MetersPerSecond),
                        MetersPerSecond),
                () -> rTeleopVel.mut_replace(
                        MathUtil.applyDeadband(-driveCtrl.getRightX(), Constants.angDriveDB)
                                * Constants.driveAngMaxVel.in(DegreesPerSecond),
                        DegreesPerSecond)));

        // Map Auto-Align
        driveCtrl.y().whileTrue(
                drivetrain.getGotoReefCmd(
                        ReefBranchOffset.MIDDLE,
                        Constants.coralXOffset,
                        Constants.coralYOffset,
                        Constants.coralROffset));

        driveCtrl.y().and(driveCtrl.rightBumper()).whileTrue(
                drivetrain.getGotoReefCmd(
                        ReefBranchOffset.RIGHT,
                        Constants.coralXOffset,
                        Constants.coralYOffset,
                        Constants.coralROffset));

        driveCtrl.y().and(driveCtrl.leftBumper()).whileTrue(
                drivetrain.getGotoReefCmd(
                        ReefBranchOffset.LEFT,
                        Constants.coralXOffset,
                        Constants.coralYOffset,
                        Constants.coralROffset));
    }

    /**
     * Initialize controls for the coral arm, elevator, and roller
     */
    private final void initCoralCtrl() {
        // Map ArmElev Presets
        opCtrl.a().onTrue(armElevCtrl.getGotoIntakeCmd());
        opCtrl.b().onTrue(armElevCtrl.getGotoL2());
        opCtrl.x().onTrue(armElevCtrl.getGotoL3());
        opCtrl.y().onTrue(armElevCtrl.getGotoL4());
        opCtrl.pov(90).onTrue(armElevCtrl.getGotoLowAlgae());
        opCtrl.pov(270).onTrue(armElevCtrl.getGotoHighAlgae());

        // Map Coral Gripper Controls
        opCtrl.leftTrigger().or(driveCtrl.rightBumper()).whileTrue(coralRoller.getEjectCmd());

        opCtrl.rightTrigger().whileTrue(coralRoller.getReverseCmd());

        driveCtrl.leftTrigger().whileTrue(coralRoller.getIntakeCmd());

    }

    /**
     * Initializes controls for the algae arm and roller
     */
    private final void initAlgaeCtrl() {
        // Map Algae arm controls
        opCtrl.pov(0).onTrue(algaeArm.getPosPresetCmd("Extended"));
        opCtrl.pov(180).onTrue(algaeArm.getPosPresetCmd("Home"));

        // Map Algae Roller Controls
        opCtrl.rightBumper().whileTrue(algaeRoller.getEjectCmd());
        opCtrl.leftBumper().whileTrue(algaeRoller.getIntakeCmd());
    }

    /**
     * Initialize controls for the climber
     */
    private final void initClimberCtrl() {
        driveCtrl.start().or(opCtrl.start()).whileTrue(climber.getAutoExtendCmd());
        driveCtrl.back().or(opCtrl.back()).whileTrue(climber.getRetractCmd());
    }

    /**
     * Initialize all named commands for PathPlanner
     */
    private void initNamedCommands() {
        // Drivetrain Named Commands
        NamedCommands.registerCommand("rightBranchAlign",
                drivetrain.getGotoReefCmd(
                        ReefBranchOffset.RIGHT,
                        Constants.autoAlignLinTol,
                        Constants.autoAlignAngTol,
                        Constants.coralXOffset,
                        Constants.coralYOffset,
                        Constants.coralROffset));
        NamedCommands.registerCommand("leftBranchAlign",
                drivetrain.getGotoReefCmd(
                        ReefBranchOffset.LEFT,
                        Constants.autoAlignLinTol,
                        Constants.autoAlignAngTol,
                        Constants.coralXOffset,
                        Constants.coralYOffset,
                        Constants.coralROffset));
        NamedCommands.registerCommand("driveDoNothing",
                drivetrain.getVelocityCmd(MetersPerSecond.zero(), MetersPerSecond.zero(),
                        DegreesPerSecond.zero()));
        NamedCommands.registerCommand("rightBranchNoFinish",
                drivetrain.getGotoReefCmd(
                        ReefBranchOffset.RIGHT,
                        Constants.coralXOffset,
                        Constants.coralYOffset,
                        Constants.coralROffset));
        NamedCommands.registerCommand("leftBranchNoFinish",
                drivetrain.getGotoReefCmd(
                        ReefBranchOffset.LEFT,
                        Constants.coralXOffset,
                        Constants.coralYOffset,
                        Constants.coralROffset));

        // ArmElev named Commands
        NamedCommands.registerCommand("goToIntakeCommand", armElevCtrl.getGotoIntakeCmd());
        NamedCommands.registerCommand("goToL1Command", armElevCtrl.getGotoL1());
        NamedCommands.registerCommand("goToL2Command", armElevCtrl.getGotoL2());
        NamedCommands.registerCommand("goToL3Command", armElevCtrl.getGotoL3());
        NamedCommands.registerCommand("goToL4Command", armElevCtrl.getGotoL4());
        NamedCommands.registerCommand("goToLowAlgaeCommand", armElevCtrl.getGotoLowAlgae());
        NamedCommands.registerCommand("goToHighAlgaeCommand", armElevCtrl.getGotoHighAlgae());

        // Coral Gripper Named Commands
        NamedCommands.registerCommand("coralPresentCommand",
                coralRoller.getWaitForCoralAtIntakeCmd(Inches.zero()));
        NamedCommands.registerCommand("coralNotPresentCommand",
                coralRoller.getWaitForCoralNotAtIntakeCmd(Inches.zero()));
        NamedCommands.registerCommand("coralInEndEffector",
                coralRoller.getWaitForCoralInGripperCmd(Inches.zero()));
        NamedCommands.registerCommand("ejectCommand", coralRoller.getAutoEjectCmd(Inches.zero()));
        NamedCommands.registerCommand("intakeCommand", coralRoller.getAutoIntakeCmd(Inches.zero()));

        // Elevator Name Commands
        NamedCommands.registerCommand("elevatorHoldCommand", elevator.getHoldPosCmd());
        NamedCommands.registerCommand("elevIntakePos", elevator.getPosPresetCmd("Intake"));
        NamedCommands.registerCommand("elevL4Pos", elevator.getPosPresetCmd("L4"));

        // Coral Arm Named Commands
        NamedCommands.registerCommand("armHoldCommand", coralArm.getHoldPosCmd());
        NamedCommands.registerCommand("armIntakeAngle", coralArm.getPosPresetCmd("Intake"));
    }

    /**
     * Initializes path planner
     */
    private void initPathPlanner() {
        PathPlanner.initPathPlanner(drivetrain);
        // TODO Put auto chooser on driver station
    }

    /**
     * Gets the currently select auton command. If no command is selected, null is
     * returned.
     */
    public Command getSelectedAuton() {
        return PathPlanner.getAutoChooser().getSelected();
    }
}
