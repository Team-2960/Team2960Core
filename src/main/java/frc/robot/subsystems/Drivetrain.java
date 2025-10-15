package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2960.helper.AngleUtil;
import frc.lib2960.helper.RobotFeature;
import frc.lib2960.subsystem.drivetrain.swerve.NavXSwerveDrive;
import frc.lib2960.subsystem.drivetrain.swerve.NavXSwerveDriveConfig;
import frc.lib2960.subsystem.drivetrain.swerve.SwerveModuleBase;
import frc.robot.Constants;

public class Drivetrain extends NavXSwerveDrive {

    private class GotoReefCmd extends Command {
        private final String faceBranch;
        private final Optional<Distance> linTol;
        private final Optional<Angle> angTol;
        private final RobotFeature feature;
        private Pose2d target;

        /**
         * Constructor.
         * 
         * @param faceBranch Reef branch offset
         */
        public GotoReefCmd(String faceBranch) {
            this.faceBranch = faceBranch;
            this.linTol = Optional.empty();
            this.angTol = Optional.empty();
            this.feature = RobotFeature.origin;
        }

        /**
         * Constructor
         * 
         * @param faceBranch Reef branch offset
         * @param feature    Robot feature offset
         */
        public GotoReefCmd(String faceBranch, RobotFeature feature) {
            this.faceBranch = faceBranch;
            this.linTol = Optional.empty();
            this.angTol = Optional.empty();
            this.feature = feature;
        }

        /**
         * Constructor
         * 
         * @param faceBranch Reef branch offset
         * @param linTol     target pose linear tolerance
         * @param angTol     target pose angular tolerance
         */
        public GotoReefCmd(String faceBranch, Distance linTol, Angle angTol) {
            this.faceBranch = faceBranch;
            this.linTol = Optional.of(linTol);
            this.angTol = Optional.of(angTol);
            this.feature = RobotFeature.origin;
        }

        /**
         * Constructor
         * 
         * @param faceBranch Reef branch offset
         * @param linTol     target pose linear tolerance
         * @param angTol     target pose angular tolerance
         * @param feature    Robot feature offset
         */
        public GotoReefCmd(String faceBranch, Distance linTol, Angle angTol, RobotFeature feature) {
            this.faceBranch = faceBranch;
            this.linTol = Optional.of(linTol);
            this.angTol = Optional.of(angTol);
            this.feature = feature;
        }

        /**
         * Captures the target pose
         */
        @Override
        public void initialize() {
            target = Constants.reefFaces.get(faceBranch).get().pose();
        }

        /**
         * Updates the gotoPose method
         */
        @Override
        public void execute() {
            gotoPose(target, feature);
        }

        /**
         * Checks if tolerances are present and send the command once the robot is at
         * the target pose
         */
        @Override
        public boolean isFinished() {
            return linTol.isPresent() && Drivetrain.this.atTarget(target.getTranslation(), linTol.get()) &&
                    angTol.isPresent()
                    && Drivetrain.this.atTarget(AngleUtil.toUnits(target.getRotation()), angTol.get());
        }
    }

    /**
     * Constructor
     * 
     * @param config  Swerve drive configuration
     * @param modules Swerve module objects
     */
    public Drivetrain(NavXSwerveDriveConfig config, SwerveModuleBase... modules) {
        super(config, modules);
    }

    /**
     * Gets a new command to goto a position on the nearest reef face
     * 
     * @param faceBranch Reef face branch name
     * @return new command to goto a position on the nearest reef face
     */
    public Command getGotoReefCmd(String faceBranch) {
        return new GotoReefCmd(faceBranch);
    }

    /**
     * Gets a new command to goto a position on the nearest reef face
     * 
     * @param faceBranch Reef face branch name
     * @param feature    Robot feature offset
     * @return new command to goto a position on the nearest reef face
     */
    public Command getGotoReefCmd(String faceBranch, RobotFeature feature) {
        return new GotoReefCmd(faceBranch, feature);
    }

    /**
     * Gets a new command to goto a position on the nearest reef face
     * 
     * @param faceBranch Reef face branch name
     * @param linTol     target pose linear tolerance
     * @param angTol     target pose angular tolerance
     * @return new command to goto a position on the nearest reef face
     */
    public Command getGotoReefCmd(String faceBranch, Distance linTol, Angle angTol) {
        return new GotoReefCmd(faceBranch, linTol, angTol);
    }

    /**
     * Gets a new command to goto a position on the nearest reef face
     * 
     * @param faceBranch Reef face branch name
     * @param linTol     target pose linear tolerance
     * @param angTol     target pose angular tolerance
     * @param feature    Robot feature offset
     * @return new command to goto a position on the nearest reef face
     */
    public Command getGotoReefCmd(String faceBranch, Distance linTol, Angle angTol, RobotFeature feature) {
        return new GotoReefCmd(faceBranch, linTol, angTol, feature);
    }
}
