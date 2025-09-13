
package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib2960.helper.PathPlanner;
import frc.lib2960.subsystem.drivetrain.swerve.NavXSwerveDrive;
import frc.lib2960.subsystem.drivetrain.swerve.RevFlexMaxSwerveModule;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.AlgaeRoller;
import frc.robot.subsystems.ArmElevControl;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.CoralRoller;
import frc.robot.subsystems.Elevator;

/**
 * Class to contain all the robot subsystems and items that are not specific to
 * a single or group of subsystems
 */
public class RobotContainer {

    // Subsystems
    public static final RevFlexMaxSwerveModule[] swerveModules = {
            new RevFlexMaxSwerveModule(Constants.swerveModuleCommonConfig, Constants.lfConfig),
            new RevFlexMaxSwerveModule(Constants.swerveModuleCommonConfig, Constants.rfConfig),
            new RevFlexMaxSwerveModule(Constants.swerveModuleCommonConfig, Constants.lrConfig),
            new RevFlexMaxSwerveModule(Constants.swerveModuleCommonConfig, Constants.rrConfig),
    };

    public static final NavXSwerveDrive drivetrain = new NavXSwerveDrive(Constants.swerveDriveConfig, swerveModules);
    public static final CoralArm coralArm = new CoralArm(Constants.coralArmConfig, Constants.coralArmMotorConfig);
    public static final CoralRoller coralRoller = new CoralRoller(Constants.coralRollerConfig);
    public static final Elevator elevator = new Elevator(Constants.elevatorConfig, Constants.elevatorMotorConfig);
    public static final ArmElevControl armElevControl = new ArmElevControl(coralArm, elevator, Constants.coralArmPosTol,
            Constants.elevatorPosTol);
    public static final AlgaeArm algaeArm = new AlgaeArm(Constants.algaeArmConfig, Constants.algaeArmMotorConfig);
    public static final AlgaeRoller algaeRoller = new AlgaeRoller(Constants.algaeRollerConfig);
    public static final Climber climber = new Climber(Constants.climberConfig);

    // Joysticks
    public static final CommandXboxController driveCtrl = new CommandXboxController(0);
    public static final CommandXboxController opCtrl = new CommandXboxController(1);

    // Helper Units
    private static final MutLinearVelocity xTeleopVel = MetersPerSecond.mutable(0);
    private static final MutLinearVelocity yTeleopVel = MetersPerSecond.mutable(0);
    private static final MutAngularVelocity rTeleopVel = DegreesPerSecond.mutable(0);

    // TODO Implement AprilTag vision
    // TODO Map Joysticks

    /**
     * Static initializer
     */
    static {
        initControls();
        initPathPlanner();
    }

    private static void initControls() {
        // Init Drivetrain controls
        drivetrain.setDefaultCommand(drivetrain.getVelocityControlCmd(
                () -> xTeleopVel.mut_replace(-driveCtrl.getLeftX() * Constants.driveLinMaxVel.in(MetersPerSecond),
                        MetersPerSecond),
                () -> yTeleopVel.mut_replace(-driveCtrl.getLeftY() * Constants.driveLinMaxVel.in(MetersPerSecond),
                        MetersPerSecond),
                () -> rTeleopVel.mut_replace(-driveCtrl.getRightX() * Constants.driveAngMaxVel.in(DegreesPerSecond),
                        DegreesPerSecond)));

    }

    /**
     * Initializes path planner
     */
    private static void initPathPlanner() {
        PathPlanner.initPathPlanner(drivetrain);
        // TODO Put auto chooser on driver station
    }
}
