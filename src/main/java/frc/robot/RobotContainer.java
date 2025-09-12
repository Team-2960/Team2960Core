
package frc.robot;

import frc.lib2960.helper.PathPlanner;
import frc.lib2960.subsystem.drivetrain.swerve.NavXSwerveDrive;
import frc.lib2960.subsystem.drivetrain.swerve.RevFlexMaxSwerveModule;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.AlgaeRoller;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.CoralRoller;
import frc.robot.subsystems.Elevator;

/**
 * Class to contain all the robot subsystems and items that are not specific to
 * a single or group of subsystems
 */
public class RobotContainer {
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
    public static final AlgaeArm algaeArm = new AlgaeArm(Constants.algaeArmConfig, Constants.algaeArmMotorConfig);
    public static final AlgaeRoller algaeRoller = new AlgaeRoller(Constants.algaeRollerConfig);
    public static final Climber climber = new Climber(Constants.climberConfig);
    
    // TODO Implement ArmElevControl
    // TODO Implement AprilTag vision
    // TODO Map Joysticks

    /**
     * Static initializer
     */
    static {   
        initPathPlanner();
    }

    /**
     * Initializes path planner
     */
    private static void initPathPlanner() {
        PathPlanner.initPathPlanner(drivetrain);
        // TODO Put auto chooser on driver station
    }
}
