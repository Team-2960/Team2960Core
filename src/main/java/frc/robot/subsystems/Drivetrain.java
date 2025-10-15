package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2960.helper.AngleUtil;
import frc.lib2960.subsystem.drivetrain.swerve.NavXSwerveDrive;
import frc.lib2960.subsystem.drivetrain.swerve.NavXSwerveDriveConfig;
import frc.lib2960.subsystem.drivetrain.swerve.SwerveModuleBase;
import frc.robot.Constants;

public class Drivetrain extends NavXSwerveDrive {

    private class GotoReefCmd extends Command {
        private final String faceBranch;
        private final Optional<Distance> linTol;
        private final Optional<Angle> angTol;
        private final Distance xOFfset;
        private final Distance yOffset;
        private final Angle rOffset;
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
            this.xOFfset = Meters.zero();
            this.yOffset = Meters.zero();
            this.rOffset = Degrees.zero();
        }

        /**
         * Constructor
         * 
         * @param faceBranch Reef branch offset
         * @param xOffset    Robot center x offset
         * @param yOffset    Robot center y offset
         * @param rOffset    Robot rotation offset;
         */
        public GotoReefCmd(String faceBranch, Distance xOFfset, Distance yOffset, Angle rOffset) {
            this.faceBranch = faceBranch;
            this.linTol = Optional.empty();
            this.angTol = Optional.empty();
            this.xOFfset = xOFfset;
            this.yOffset = yOffset;
            this.rOffset = rOffset;
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
            this.xOFfset = Meters.zero();
            this.yOffset = Meters.zero();
            this.rOffset = Degrees.zero();
        }

        /**
         * Constructor
         * 
         * @param faceBranch Reef branch offset
         * @param linTol     target pose linear tolerance
         * @param angTol     target pose angular tolerance
         * @param xOffset    Robot center x offset
         * @param yOffset    Robot center y offset
         * @param rOffset    Robot rotation offset;
         */
        public GotoReefCmd(String faceBranch, Distance linTol, Angle angTol, Distance xOFfset, Distance yOffset,
                Angle rOffset) {
            this.faceBranch = faceBranch;
            this.linTol = Optional.of(linTol);
            this.angTol = Optional.of(angTol);
            this.xOFfset = xOFfset;
            this.yOffset = yOffset;
            this.rOffset = rOffset;
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
            gotoPose(target, xOFfset, yOffset, rOffset);
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
     * @param faceBranch  Reef face branch name
     * @return new command to goto a position on the nearest reef face
     */
    public Command getGotoReefCmd(String faceBranch) {
        return new GotoReefCmd(faceBranch);
    }

    /**
     * Gets a new command to goto a position on the nearest reef face
     * 
     * @param faceBranch  Reef face branch name
     * @param xOffset Robot center x offset
     * @param yOffset Robot center y offset
     * @param rOffset Robot rotation offset;
     * @return new command to goto a position on the nearest reef face
     */
    public Command getGotoReefCmd(String faceBranch, Distance xOFfset, Distance yOffset, Angle rOffset) {
        return new GotoReefCmd(faceBranch, xOFfset, yOffset, rOffset);
    }

    /**
     * Gets a new command to goto a position on the nearest reef face
     * 
     * @param faceBranch  Reef face branch name
     * @param linTol target pose linear tolerance
     * @param angTol target pose angular tolerance
     * @return new command to goto a position on the nearest reef face
     */
    public Command getGotoReefCmd(String faceBranch, Distance linTol, Angle angTol) {
        return new GotoReefCmd(faceBranch, linTol, angTol);
    }

    /**
     * Gets a new command to goto a position on the nearest reef face
     * 
     * @param faceBranch  Reef face branch name
     * @param linTol  target pose linear tolerance
     * @param angTol  target pose angular tolerance
     * @param xOffset Robot center x offset
     * @param yOffset Robot center y offset
     * @param rOffset Robot rotation offset;
     * @return new command to goto a position on the nearest reef face
     */
    public Command getGotoReefCmd(String faceBranch, Distance linTol, Angle angTol, Distance xOFfset,
            Distance yOffset, Angle rOffset) {
        return new GotoReefCmd(faceBranch, linTol, angTol, xOFfset, yOffset, rOffset);
    }
}
