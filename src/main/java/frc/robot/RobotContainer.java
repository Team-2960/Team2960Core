
package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2960.helper.PathPlanner;
import frc.lib2960.subsystem.drivetrain.swerve.NavXSwerveDrive;
import frc.lib2960.subsystem.drivetrain.swerve.RevFlexMaxSwerveModule;
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
    public final RevFlexMaxSwerveModule[] swerveModules = {
            new RevFlexMaxSwerveModule(Constants.swerveModuleCommonConfig, Constants.lfConfig),
            new RevFlexMaxSwerveModule(Constants.swerveModuleCommonConfig, Constants.rfConfig),
            new RevFlexMaxSwerveModule(Constants.swerveModuleCommonConfig, Constants.lrConfig),
            new RevFlexMaxSwerveModule(Constants.swerveModuleCommonConfig, Constants.rrConfig),
    };

    public final Drivetrain drivetrain = new Drivetrain(Constants.swerveDriveConfig,
            swerveModules);
    public final CoralArm coralArm = new CoralArm(Constants.coralArmConfig, Constants.coralArmMotorConfig);
    public final CoralRoller coralRoller = new CoralRoller(Constants.coralRollerConfig);
    public final Elevator elevator = new Elevator(Constants.elevatorConfig, Constants.elevatorMotorConfig);
    public final ArmElevControl armElevControl = new ArmElevControl(coralArm, elevator,
            Constants.coralArmPosTol,
            Constants.elevatorPosTol);
    public final AlgaeArm algaeArm = new AlgaeArm(Constants.algaeArmConfig, Constants.algaeArmMotorConfig);
    public final AlgaeRoller algaeRoller = new AlgaeRoller(Constants.algaeRollerConfig);
    public final Climber climber = new Climber(Constants.climberConfig);
    public final  

    // Joysticks
    public final CommandXboxController driveCtrl = new CommandXboxController(Constants.driverCtrlID);
    public final CommandXboxController opCtrl = new CommandXboxController(Constants.opCtrlID);

    // Helper Units
    private final MutLinearVelocity xTeleopVel = MetersPerSecond.mutable(0);
    private final MutLinearVelocity yTeleopVel = MetersPerSecond.mutable(0);
    private final MutAngularVelocity rTeleopVel = DegreesPerSecond.mutable(0);

    // TODO Implement AprilTag vision
    // TODO Map Joysticks

    /**
     * initializer
     */
    public RobotContainer() {
        // Init Driver Controls
        initDrivetrainCtrl();
        initCoralGripperCtrl();

        // Init PathPlanner
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

    private final void initCoralCtrl() {
        // Map ArmElev Presets
        opCtrl.y().onTrue(Elev)
    }

    /**
     * Initializes path planner
     */
    private void initPathPlanner() {
        PathPlanner.initPathPlanner(drivetrain);
        // TODO Put auto chooser on driver station
    }

}
