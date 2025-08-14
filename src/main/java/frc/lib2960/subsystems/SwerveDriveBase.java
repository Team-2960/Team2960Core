package frc.lib2960.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2960.settings.SwerveDriveBaseSettings;
import frc.lib2960.util.LinearVelocityVector2d;

/**
 * Defines core capabilities of a swerve drive
 */
public abstract class SwerveDriveBase extends SubsystemBase {

    /**********************/
    /* Settings Variables */
    /**********************/
    protected final SwerveDriveBaseSettings settings;
    private boolean isFieldRelative;

    /*************************/
    /* Calculation Variables */
    /*************************/
    private LinearVelocityVector2d velVector = new LinearVelocityVector2d();
    private MutAngularVelocity rVelMut = RadiansPerSecond.mutable(0);

    /****************************/
    /* Driver Station Variables */
    /****************************/

    /********************/
    /* Control Commands */
    /********************/

    public class SwerveCmd extends Command {
        protected ChassisSpeeds speeds;
        protected Distance xOffset;
        protected Distance yOffset;
        protected Angle rOffset;

        public SwerveCmd() {
            this(Meters.zero(), Meters.zero(), Radians.zero());
        }

        public SwerveCmd(Distance xOffset, Distance yOffset) {
            this(xOffset, yOffset, Radians.zero());
        }

        public SwerveCmd(Angle rOffset) {
            this(Meters.zero(), Meters.zero(), rOffset);
        }

        public SwerveCmd(Distance xOffset, Distance yOffset, Angle rOffset) {
            speeds = new ChassisSpeeds();
            this.xOffset = xOffset;
            this.yOffset = yOffset;
            this.rOffset = rOffset;
        }

        @Override
        public void end(boolean interrupted) {
            speeds.vxMetersPerSecond = 0;
            speeds.vyMetersPerSecond = 0;
            speeds.omegaRadiansPerSecond = 0;
            setChassisSpeeds(speeds);
        }
    }

    /**
     * Controls the swerve drive using rate control
     */
    public class RateControlCmd extends SwerveCmd {
        private final Supplier<LinearVelocity> xVel;
        private final Supplier<LinearVelocity> yVel;
        private final Supplier<AngularVelocity> rVel;

        /**
         * Constructor
         * 
         * @param xVel Supplier for x velocity
         * @param yVel Supplier for y velocity
         * @param rVel Supplier for angular velocity
         */
        public RateControlCmd(
                Supplier<LinearVelocity> xVel,
                Supplier<LinearVelocity> yVel,
                Supplier<AngularVelocity> rVel) {
            this.xVel = xVel;
            this.yVel = yVel;
            this.rVel = rVel;
        }

        /**
         * Constructor
         * 
         * @param xVel    Supplier for x velocity
         * @param yVel    Supplier for y velocity
         * @param rVel    Supplier for angular velocity
         * @param xOffset x offset for the center of rotation
         * @param yOffset y offset for the center of rotation
         */
        public RateControlCmd(
                Supplier<LinearVelocity> xVel,
                Supplier<LinearVelocity> yVel,
                Supplier<AngularVelocity> rVel,
                Distance xOffset,
                Distance yOffset) {
            super(xOffset, yOffset);
            this.xVel = xVel;
            this.yVel = yVel;
            this.rVel = rVel;
        }

        /**
         * Updates chassis speeds
         */
        @Override
        public void execute() {
            speeds.vxMetersPerSecond = xVel.get().in(MetersPerSecond);
            speeds.vyMetersPerSecond = yVel.get().in(MetersPerSecond);
            speeds.omegaRadiansPerSecond = rVel.get().in(RadiansPerSecond);
            setChassisSpeeds(speeds, xOffset, yOffset);
        }
    }

    public class TurnToAngleCmd extends SwerveCmd {
        private final Supplier<LinearVelocity> xVel;
        private final Supplier<LinearVelocity> yVel;
        private final Angle target;
        private MutAngle angle = Radians.mutable(0);
        private Optional<Angle> tolerance = Optional.empty();

        public TurnToAngleCmd(
                Supplier<LinearVelocity> xVel,
                Supplier<LinearVelocity> yVel,
                Angle target) {

            this.xVel = xVel;
            this.yVel = yVel;
            this.target = target;
        }

        public TurnToAngleCmd(
                Supplier<LinearVelocity> xVel,
                Supplier<LinearVelocity> yVel,
                Angle target,
                Angle tolerance) {

            this(xVel, yVel, target);
            this.tolerance = Optional.of(tolerance);
        }

        public TurnToAngleCmd(
                Supplier<LinearVelocity> xVel,
                Supplier<LinearVelocity> yVel,
                Angle target,
                Distance xOffset,
                Distance yOffset,
                Angle rOffset) {

            super(xOffset, yOffset, rOffset);
            this.xVel = xVel;
            this.yVel = yVel;
            this.target = target;
        }

        public TurnToAngleCmd(
                Supplier<LinearVelocity> xVel,
                Supplier<LinearVelocity> yVel,
                Angle target,
                Angle tolerance,
                Distance xOffset,
                Distance yOffset,
                Angle rOffset) {

            this(xVel, yVel, target, xOffset, yOffset, rOffset);
            this.tolerance = Optional.of(tolerance);
        }

        @Override
        public void execute() {
            speeds.vxMetersPerSecond = xVel.get().in(MetersPerSecond);
            speeds.vyMetersPerSecond = yVel.get().in(MetersPerSecond);
            speeds.omegaRadiansPerSecond = calcRateToAngle(calcAngleOffset()).in(RadiansPerSecond);
            setChassisSpeeds(speeds, xOffset, yOffset);
        }

        @Override
        public boolean isFinished() {
            return tolerance.isPresent() && atTarget(calcAngleOffset(), tolerance.get());
        }

        private Angle calcAngleOffset() {
            angle.mut_replace(target);
            angle.mut_plus(rOffset);
            return angle;
        }
    }

    public class TurnToPointCmd extends SwerveCmd {
        private final Supplier<LinearVelocity> xVel;
        private final Supplier<LinearVelocity> yVel;
        private final Translation2d target;
        private Optional<Angle> tolerance = Optional.empty();

        private MutAngle angle = Radians.mutable(0);

        public TurnToPointCmd(
                Supplier<LinearVelocity> xVel,
                Supplier<LinearVelocity> yVel,
                Translation2d target,
                Distance xOffset,
                Distance yOffset,
                Angle rOffset) {
            super(xOffset, yOffset, rOffset);
            this.xVel = xVel;
            this.yVel = yVel;
            this.target = target;
        }

        public TurnToPointCmd(
                Supplier<LinearVelocity> xVel,
                Supplier<LinearVelocity> yVel,
                Translation2d target,
                Angle tolerance,
                Distance xOffset,
                Distance yOffset,
                Angle rOffset) {

            this(xVel, yVel, target, xOffset, yOffset, rOffset);
            this.tolerance = Optional.of(tolerance);
        }

        public TurnToPointCmd(
                Supplier<LinearVelocity> xVel,
                Supplier<LinearVelocity> yVel,
                Translation2d target) {

            this.xVel = xVel;
            this.yVel = yVel;
            this.target = target;
        }

        public TurnToPointCmd(
                Supplier<LinearVelocity> xVel,
                Supplier<LinearVelocity> yVel,
                Translation2d target,
                Angle tolerance) {

            this(xVel, yVel, target);
            this.tolerance = Optional.of(tolerance);
        }

        @Override
        public void execute() {
            speeds.vxMetersPerSecond = xVel.get().in(MetersPerSecond);
            speeds.vyMetersPerSecond = yVel.get().in(MetersPerSecond);
            speeds.omegaRadiansPerSecond = calcRateToAngle(calcAngleOffset()).in(RadiansPerSecond);
            setChassisSpeeds(speeds, xOffset, yOffset);
        }

        @Override
        public boolean isFinished() {
            return tolerance.isPresent() && atTarget(calcAngleOffset(), tolerance.get());
        }

        private Angle calcAngleOffset() {
            var cur_pos = getPostEst().getTranslation();

            angle.mut_replace(cur_pos.minus(target).getAngle().getRadians(), Radians);
            angle.mut_plus(rOffset);

            return angle;
        }
    }

    public class GotoPointCmd extends SwerveCmd {
        private final Translation2d target;
        private final Supplier<AngularVelocity> rVel;

        private Optional<Distance> tolerance = Optional.empty();

        public GotoPointCmd(
                Translation2d target,
                Supplier<AngularVelocity> rVel) {
            this.target = target;
            this.rVel = rVel;
        }

        public GotoPointCmd(
                Translation2d target,
                Supplier<AngularVelocity> rVel,
                Distance tolerance) {
            this(target, rVel);
            this.tolerance = Optional.of(tolerance);
        }

        public GotoPointCmd(
                Translation2d target,
                Supplier<AngularVelocity> rVel,
                Distance xOffset,
                Distance yOffset) {
            super(xOffset, yOffset);
            this.target = target;
            this.rVel = rVel;
        }

        public GotoPointCmd(
                Translation2d target,
                Supplier<AngularVelocity> rVel,
                Distance tolerance,
                Distance xOffset,
                Distance yOffset) {
            this(target, rVel, xOffset, yOffset);
            this.tolerance = Optional.of(tolerance);
        }

        @Override
        public void execute() {
            velVector = calcVelToPosition(target);
            speeds.vxMetersPerSecond = velVector.xVel.in(MetersPerSecond);
            speeds.vyMetersPerSecond = velVector.yVel.in(MetersPerSecond);
            speeds.omegaRadiansPerSecond = rVel.get().in(RadiansPerSecond);
            setChassisSpeeds(speeds, xOffset, yOffset);
        }

        @Override
        public boolean isFinished() {
            return tolerance.isPresent() && atTarget(target, tolerance.get());
        }
    }

    public class GotoPoseCmd extends SwerveCmd {
        private final Pose2d target;

        private Optional<Distance> linearTol = Optional.empty();
        private Optional<Angle> angleTol = Optional.empty();

        private MutAngle angle = Radians.mutable(0);

        public GotoPoseCmd(
                Pose2d target) {
            this.target = target;
        }

        public GotoPoseCmd(
                Pose2d target,
                Distance linearTol,
                Angle angleTol) {
            this(target);
            this.linearTol = Optional.of(linearTol);
            this.angleTol = Optional.of(angleTol);
        }

        public GotoPoseCmd(
                Pose2d target,
                Distance xOffset,
                Distance yOffset,
                Angle rOffset) {
            super(xOffset, yOffset, rOffset);
            this.target = target;
        }

        public GotoPoseCmd(
                Pose2d target,
                Distance linearTol,
                Angle angleTol,
                Distance xOffset,
                Distance yOffset,
                Angle rOffset) {
            this(target, xOffset, yOffset, rOffset);
            this.linearTol = Optional.of(linearTol);
            this.angleTol = Optional.of(angleTol);
        }

        @Override
        public void execute() {
            velVector = calcVelToPosition(target.getTranslation());

            speeds.vxMetersPerSecond = velVector.xVel.in(MetersPerSecond);
            speeds.vyMetersPerSecond = velVector.yVel.in(MetersPerSecond);
            speeds.omegaRadiansPerSecond = calcRateToAngle(calcAngleOffset()).in(RadiansPerSecond);

            setChassisSpeeds(speeds, xOffset, yOffset);
        }

        @Override
        public boolean isFinished() {
            return linearTol.isPresent() && atTarget(target.getTranslation(), linearTol.get()) &&
                    angleTol.isPresent() && atTarget(calcAngleOffset(), angleTol.get());

        }

        private Angle calcAngleOffset() {
            angle.mut_replace(target.getRotation().getRadians(), Radians);
            angle.mut_plus(rOffset);
            return angle;
        }
    }

    /*********************/
    /* Settings Commands */
    /*********************/

    /****************/
    /* Constructors */
    /****************/
    public SwerveDriveBase(SwerveDriveBaseSettings settings) {
        this.settings = settings;
    }

    /*******************/
    /* Control Methods */
    /*******************/

    public void setPathPlannerSpeeds(ChassisSpeeds speeds) {
        setChassisSpeeds(speeds, false);
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        setChassisSpeeds(speeds, this.isFieldRelative);
    }

    public void setChassisSpeeds(ChassisSpeeds speeds, boolean isFieldRelative) {
        setChassisSpeeds(speeds, isFieldRelative, Meters.zero(), Meters.zero());
    }

    public void setChassisSpeeds(ChassisSpeeds speeds, Distance xOffset, Distance yOffset) {
        setChassisSpeeds(speeds, this.isFieldRelative, xOffset, yOffset);
    }

    public abstract void setChassisSpeeds(ChassisSpeeds speeds, boolean isFieldRelative, Distance xOffset,
            Distance yOffset);

    public LinearVelocityVector2d calcVelToPosition(Translation2d target) {
        // TODO calculate the x and y linear velocities to reach a target position

        velVector.xVel.mut_replace(0, MetersPerSecond);
        velVector.yVel.mut_replace(0, MetersPerSecond);

        return velVector;
    }

    public AngularVelocity calcRateToAngle(Angle target) {
        // TODO calculate the angular rate to get to a target angle
        return rVelMut.mut_replace(0, RadiansPerSecond);
    }

    /******************/
    /* Access Methods */
    /******************/
    public abstract Pose2d getPostEst();

    public abstract ChassisSpeeds getChassisSpeeds();

    public abstract void resetPose(Pose2d pose);

    /********************/
    /* Settings Methods */
    /********************/
    public void setFieldRelative(boolean isFieldRelative) {
        this.isFieldRelative = isFieldRelative;
    }

    /******************/
    /* Helper Methods */
    /******************/
    public boolean atTarget(Angle target, Angle tolerance) {

        boolean result = false;

        double cur_angle = getPostEst().getRotation().getRadians();
        double target_dbl = target.in(Radians);
        double tolerance_dbl = tolerance.in(Radians);

        result = cur_angle + tolerance_dbl > target_dbl && cur_angle - tolerance_dbl < target_dbl;
        result |= cur_angle + 2 * Math.PI + tolerance_dbl > target_dbl
                && cur_angle + 2 * Math.PI - tolerance_dbl < target_dbl;
        result |= cur_angle - 2 * Math.PI + tolerance_dbl > target_dbl
                && cur_angle - 2 * Math.PI - tolerance_dbl < target_dbl;

        return result;
    }

    public boolean atTarget(Translation2d target, Angle tolerance) {

        boolean result = false;
        Pose2d cur_pose = getPostEst();

        double cur_angle = cur_pose.getRotation().getRadians();
        double target_dbl = target.minus(cur_pose.getTranslation()).getAngle().getRadians();
        double tolerance_dbl = tolerance.in(Radians);

        result = cur_angle + tolerance_dbl > target_dbl && cur_angle - tolerance_dbl < target_dbl;
        result |= cur_angle + 2 * Math.PI + tolerance_dbl > target_dbl
                && cur_angle + 2 * Math.PI - tolerance_dbl < target_dbl;
        result |= cur_angle - 2 * Math.PI + tolerance_dbl > target_dbl
                && cur_angle - 2 * Math.PI - tolerance_dbl < target_dbl;

        return result;
    }

    public boolean atTarget(Translation2d target, Distance tolerance) {
        var cur_pos = getPostEst().getTranslation();

        double distance = cur_pos.getDistance(target);

        return distance >= tolerance.in(Meters);
    }
}
