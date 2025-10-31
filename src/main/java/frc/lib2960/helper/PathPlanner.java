package frc.lib2960.helper;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.controllers.PathFollowingController;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2960.controller.PIDConfig;
import frc.lib2960.subsystem.drivetrain.Drivetrain;
import frc.lib2960.subsystem.drivetrain.HolonomicDrivetrain;
import frc.lib2960.subsystem.drivetrain.NonHolonomicDrivetrain;

public class PathPlanner {
    private static SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    /**
     * Initializes PathPlanner with a holonomic drivetrain. All Named Commands and
     * Triggers must be added before this method is called.
     * 
     * @param drivetrain Drivetrain object for PathPlanner to control
     */
    public static void initPathPlanner(HolonomicDrivetrain drivetrain) {
        initPathPlanner(
                drivetrain,
                new PPHolonomicDriveController(
                        toPIDConstants(drivetrain.getLinearPathPlannerPID()),
                        toPIDConstants(drivetrain.getAngularPathPlannerPID())));
    }

    /**
     * Initializes PathPlanner with a non-holonomic drivetrain. All Named Commands
     * and Triggers must be added before this method is called.
     * 
     * @param drivetrain Drivetrain object for PathPlanner to control
     */
    public static void initPathPlanner(NonHolonomicDrivetrain drivetrain) {
        initPathPlanner(
                drivetrain,
                new PPLTVController(drivetrain.getUpdatePeriod().in(Seconds)));
    }

    /**
     * Initializes PathPlanner. All Named Commands and Triggers must be added before
     * this method is called.
     * 
     * @param drivetrain     Drivetrain object for PathPlanner to control
     * @param pathController Path following controller
     */
    public static void initPathPlanner(Drivetrain drivetrain, PathFollowingController pathController) {

        // Configure AutoBuilder
        try {
            AutoBuilder.configure(
                    drivetrain::getPoseEst,
                    drivetrain::resetPose,
                    drivetrain::getRobotRelativeSpeeds,
                    (speeds, feedforwards) -> drivetrain.setPathPlannerSpeeds(speeds),
                    pathController,
                    RobotConfig.fromGUISettings(),
                    FieldUtil::isRedAlliance,
                    drivetrain);
        } catch (Exception e) {
            e.printStackTrace();
            // TODO Improve error handling
        }

        // Create autoChooser
        autoChooser = AutoBuilder.buildAutoChooser();
    }

    /**
     * Retrieves an auto chooser for smart dashboard
     * 
     * @return auto chooser for smart dashboard
     */
    public static SendableChooser<Command> getAutoChooser() {
        return autoChooser;
    }

    /**
     * Converts a lib2960 PIDConfig to a PathPlanner PIDConstants
     * 
     * @param config
     * @return
     */
    public static PIDConstants toPIDConstants(PIDConfig config) {
        return new PIDConstants(config.kP, config.kI, config.kD);
    }
}
