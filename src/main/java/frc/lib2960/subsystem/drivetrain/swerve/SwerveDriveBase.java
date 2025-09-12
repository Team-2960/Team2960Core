package frc.lib2960.subsystem.drivetrain.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.BooleanSupplier;
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
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib2960.controller.AngularController;
import frc.lib2960.controller.LinearController;
import frc.lib2960.subsystem.drivetrain.HolonomicDrivetrain;
import frc.lib2960.config.controller.PIDConfig;
import frc.lib2960.config.subsystem.SwerveDriveCommonConfig;
import frc.lib2960.helper.AngleUtil;

/**
 * Defines core capabilities of a swerve drive
 */
public abstract class SwerveDriveBase implements HolonomicDrivetrain {
    // TODO Implement SysID
    // TODO Implement Telemetry
    // TODO Implement Logging

    /**********************/
    /* Config Variables */
    /**********************/
    protected final SwerveDriveCommonConfig config;
    private boolean isFieldRelative;

    private final LinearController linearCtrl;
    private final AngularController angleCtrl;

    /*************************/
    /* Calculation Variables */
    /*************************/
    private final MutDistance posErrorCalc = Meters.mutable(0);
    private final MutLinearVelocity velMagCalc = MetersPerSecond.mutable(0);
    private final MutLinearVelocity xVelCalc = MetersPerSecond.mutable(0);
    private final MutLinearVelocity yVelCalc = MetersPerSecond.mutable(0);
    private final MutAngle angleTarget = Radians.mutable(0);

    private final MutAngle angleCalc = Radians.mutable(0);
    private final MutAngularVelocity angleVelCalc = RadiansPerSecond.mutable(0);

    private final ChassisSpeeds targetSpeeds = new ChassisSpeeds();

    /*************************/
    /* Suffleboard Variables */
    /*************************/

    /****************/
    /* Constructors */
    /****************/

    /**
     * Constructor
     * 
     * @param config swerve drive configuration
     */
    public SwerveDriveBase(SwerveDriveCommonConfig config) {
        this.config = config;
        this.linearCtrl = new LinearController(config.linearCtrlConfig);
        this.angleCtrl = new AngularController(config.angleCtrlConfig);
    }

    /*******************/
    /* Control Methods */
    /*******************/

    /**
     * Updates the swerve drive base to the given target pose
     * 
     * @param target target pose
     */
    public final void gotoPose(Pose2d target) {
        gotoPose(target, Meters.zero(), Meters.zero(), Radians.zero());
    }

    /**
     * Updates the swerve drive base to the given target pose
     * 
     * @param target  target pose
     * @param rOffset Robot angle offset
     */
    public final void gotoPose(Pose2d target, Angle rOffset) {
        gotoPose(target, Meters.zero(), Meters.zero(), rOffset);
    }

    /**
     * Updates the swerve drive base to the given target pose
     * 
     * @param target  target pose
     * @param xOffset Robot coordinate x offset for the center of rotation
     * @param yOffset Robot coordinate y offset for the center of rotation
     */
    public final void gotoPose(Pose2d target, Distance xOffset, Distance yOffset) {
        gotoPose(target, xOffset, xOffset, Radians.zero());
    }

    /**
     * Updates the swerve drive base to the given target pose
     * 
     * @param target  target pose
     * @param xOffset Robot coordinate x offset for the center of rotation
     * @param yOffset Robot coordinate y offset for the center of rotation
     * @param rOffset Robot angle offset
     */
    public void gotoPose(Pose2d target, Distance xOffset, Distance yOffset, Angle rOffset) {
        calcVelToPosition(target.getTranslation(), xVelCalc, yVelCalc);
        angleTarget.mut_replace(target.getRotation().getRadians(), Radians);
        turnToAngle(xVelCalc, yVelCalc, angleTarget, xOffset, yOffset, rOffset);
    }

    /**
     * Updates the swerve drive base to point towarwd a given target
     * 
     * @param xVel   Target x velocity
     * @param yVel   Target y velocity
     * @param target Target point
     */
    public final void turnToPoint(LinearVelocity xVel, LinearVelocity yVel, Translation2d target) {
        turnToPoint(xVel, yVel, target, Meters.zero(), Meters.zero(), Radians.zero());
    }

    /**
     * Updates the swerve drive base to point towarwd a given target
     * 
     * @param xVel    Target x velocity
     * @param yVel    Target y velocity
     * @param target  Target point
     * @param rOffset Robot angle offset
     */
    public final void turnToPoint(LinearVelocity xVel, LinearVelocity yVel, Translation2d target, Angle rOffset) {
        turnToPoint(xVel, yVel, target, Meters.zero(), Meters.zero(), rOffset);
    }

    /**
     * Updates the swerve drive base to point towarwd a given target
     * 
     * @param xVel    Target x velocity
     * @param yVel    Target y velocity
     * @param target  Target point
     * @param xOffset Robot coordinate x offset for the center of rotation
     * @param yOffset Robot coordinate y offset for the center of rotation
     */
    public final void turnToPoint(LinearVelocity xVel, LinearVelocity yVel, Translation2d target, Distance xOffset,
            Distance yOffset) {
        turnToPoint(xVel, yVel, target, xOffset, yOffset, Radians.zero());
    }

    /**
     * Updates the swerve drive base to point towarwd a given target
     * 
     * @param xVel    Target x velocity
     * @param yVel    Target y velocity
     * @param target  Target point
     * @param xOffset Robot coordinate x offset for the center of rotation
     * @param yOffset Robot coordinate y offset for the center of rotation
     * @param rOffset Robot angle offset
     */
    public void turnToPoint(LinearVelocity xVel, LinearVelocity yVel, Translation2d target, Distance xOffset,
            Distance yOffset, Angle rOffset) {

        calcAngleToPoint(target, angleTarget);
        turnToAngle(xVel, yVel, angleTarget, xOffset, yOffset, rOffset);
    }

    /**
     * Updates the swerve drive base to turn to a given angle
     * 
     * @param xVel   Target x velocity
     * @param yVel   Target y velocity
     * @param target Target angle
     */
    public final void turnToAngle(LinearVelocity xVel, LinearVelocity yVel, Angle target) {
        turnToAngle(xVel, yVel, target, Meters.zero(), Meters.zero(), Radians.zero());
    }

    /**
     * Updates the swerve drive base to turn to a given angle
     * 
     * @param xVel    Target x velocity
     * @param yVel    Target y velocity
     * @param target  Target angle
     * @param xOffset Robot coordinate x offset for the center of rotation
     * @param yOffset Robot coordinate y offset for the center of rotation
     */
    public final void turnToAngle(LinearVelocity xVel, LinearVelocity yVel, Angle target, Distance xOffset,
            Distance yOffset) {
        turnToAngle(xVel, yVel, target, xOffset, yOffset, Radians.zero());
    }

    /**
     * Updates the swerve drive base to turn to a given angle
     * 
     * @param xVel    Target x velocity
     * @param yVel    Target y velocity
     * @param target  Target angle
     * @param rOffset Robot angle offset
     */
    public final void turnToAngle(LinearVelocity xVel, LinearVelocity yVel, Angle target, Angle rOffset) {
        turnToAngle(xVel, yVel, target, Meters.zero(), Meters.zero(), rOffset);
    }

    /**
     * Updates the swerve drive base to turn to a given angle
     * 
     * @param xVel    Target x velocity
     * @param yVel    Target y velocity
     * @param target  Target angle
     * @param xOffset Robot coordinate x offset for the center of rotation
     * @param yOffset Robot coordinate y offset for the center of rotation
     * @param rOffset Robot angle offset
     */
    public void turnToAngle(LinearVelocity xVel, LinearVelocity yVel, Angle target, Distance xOffset,
            Distance yOffset, Angle rOffset) {

        calcVelToAngle(target, angleVelCalc);
        targetSpeeds.vxMetersPerSecond = xVel.in(MetersPerSecond);
        targetSpeeds.vyMetersPerSecond = yVel.in(MetersPerSecond);
        targetSpeeds.omegaRadiansPerSecond = angleVelCalc.in(RadiansPerSecond);
    }

    /**
     * Sets the target chassis speeds of the robot. Center of rotation is the
     * default center. isFieldRelative uses the value set in setFieldRelative
     * method.
     * 
     * @param xVel target x velocity
     * @param yVel target y velocity
     * @param rVel target rotation velocity
     */
    public final void setChassisSpeeds(LinearVelocity xVel, LinearVelocity yVel, AngularVelocity rVel) {
        setChassisSpeeds(xVel, yVel, rVel, this.isFieldRelative, Meters.zero(), Meters.zero());
    }

    /**
     * Sets the target chassis speeds of the robot. Center of rotation is the
     * default center.
     * 
     * @param xVel            target x velocity
     * @param yVel            target y velocity
     * @param rVel            target rotation velocity
     * @param isFieldRelative overrides the value set in setFieldRelative
     */
    public final void setChassisSpeeds(LinearVelocity xVel, LinearVelocity yVel, AngularVelocity rVel,
            boolean isFieldRelative) {
        setChassisSpeeds(xVel, yVel, rVel, isFieldRelative, Meters.zero(), Meters.zero());
    }

    /**
     * Sets the target chassis speeds of the robot. isFieldRelative uses the value
     * set in setFieldRelative method.
     * 
     * @param xVel    target x velocity
     * @param yVel    target y velocity
     * @param rVel    target rotation velocity
     * @param xOffset Robot coordinate x offset for the center of rotation
     * @param yOffset Robot coordinate y offset for the center of rotation
     */
    public final void setChassisSpeeds(LinearVelocity xVel, LinearVelocity yVel, AngularVelocity rVel, Distance xOffset,
            Distance yOffset) {
        setChassisSpeeds(xVel, yVel, rVel, this.isFieldRelative, xOffset, yOffset);
    }

    /**
     * Sets the target chassis speeds of the robot. isFieldRelative uses the value
     * set in setFieldRelative method.
     * 
     * @param xVel    target x velocity
     * @param yVel    target y velocity
     * @param rVel    target rotation velocity
     * @param xOffset Robot coordinate x offset for the center of rotation
     * @param yOffset Robot coordinate y offset for the center of rotation
     */
    public final void setChassisSpeeds(LinearVelocity xVel, LinearVelocity yVel, AngularVelocity rVel,
            boolean isFieldRelative, Distance xOffset, Distance yOffset) {
        setChassisSpeeds(
                new ChassisSpeeds(
                        xVel.in(MetersPerSecond),
                        yVel.in(MetersPerSecond),
                        rVel.in(RadiansPerSecond)),
                this.isFieldRelative,
                xOffset,
                yOffset);
    }

    /**
     * Sets the target chassis speeds of the robot. Center of rotation is the
     * default center. isFieldRelative uses the value set in setFieldRelative
     * method.
     * 
     * @param speeds target chassis speeds
     */
    public final void setChassisSpeeds(ChassisSpeeds speeds) {
        setChassisSpeeds(speeds, this.isFieldRelative);
    }

    /**
     * Sets the target chassis speeds of the robot. Center of rotation is the
     * default center.
     * 
     * @param speeds          target chassis speeds
     * @param isFieldRelative overrides the value set in setFieldRelative
     */
    public final void setChassisSpeeds(ChassisSpeeds speeds, boolean isFieldRelative) {
        setChassisSpeeds(speeds, isFieldRelative, Meters.zero(), Meters.zero());
    }

    /**
     * Sets the target chassis speeds of the robot. isFieldRelative uses the value
     * set in setFieldRelative method.
     * 
     * @param speeds  target chassis speeds
     * @param xOffset Robot coordinate x offset for the center of rotation
     * @param yOffset Robot coordinate y offset for the center of rotation
     */
    public final void setChassisSpeeds(ChassisSpeeds speeds, Distance xOffset, Distance yOffset) {
        setChassisSpeeds(speeds, this.isFieldRelative, xOffset, yOffset);
    }

    /**
     * Calculates the target linear velocities to reach a target position
     * 
     * @param target  target position
     * @param xResult mutable linear velocity for x result
     * @param yResult mutable linear velocity for y result
     */
    public void calcVelToPosition(Translation2d target, MutLinearVelocity xResult, MutLinearVelocity yResult) {
        Pose2d curPose = getPoseEst();
        ChassisSpeeds speeds = getFieldRelativeSpeeds();
        Translation2d error = target.minus(curPose.getTranslation());

        posErrorCalc.mut_replace(target.getDistance(curPose.getTranslation()), Meters);
        double velAng = error.getAngle().getRadians();

        double velMag = Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));
        velMagCalc.mut_replace(velMag, MetersPerSecond);

        linearCtrl.updateVelocity(Meters.zero(), velMagCalc, posErrorCalc, velMagCalc);

        velMag = velMagCalc.in(MetersPerSecond);

        xResult.mut_replace(velMag * Math.cos(velAng), MetersPerSecond);
        yResult.mut_replace(velMag * Math.sin(velAng), MetersPerSecond);
    }

    /**
     * Calculates the angle to a a given point
     * 
     * @param target target point
     * @param result mutable angle for storing the result
     */
    public void calcAngleToPoint(Translation2d target, MutAngle result) {
        Pose2d curPose = getPoseEst();
        result.mut_replace(target.minus(curPose.getTranslation()).getAngle().getRadians(), Radians);
    }

    /**
     * Calculates the angular velocity to reach a target angle
     * 
     * @param target target angle
     * @param result mutable angular velocity for storing the result
     */
    public void calcVelToAngle(Angle target, MutAngularVelocity result) {
        angleCalc.mut_replace(getPoseEst().getRotation().getRadians(), Radians);
        angleVelCalc.mut_replace(getFieldRelativeSpeeds().omegaRadiansPerSecond, RadiansPerSecond);

        angleCtrl.updateVelocity(angleCalc, angleVelCalc, target, result);
    }

    /******************/
    /* Access Methods */
    /******************/

    /**
     * Gets the Linear PID parameters for PathPlanner
     * 
     * @return Linear PID parameters for PathPlanner
     */
    @Override
    public PIDConfig getLinearPathPlannerPID() {
        return config.linearPPPID;
    }

    /**
     * Gets the Angular PID parameters for PathPlanner
     * 
     * @return Angular PID parameters for PathPlanner
     */
    @Override
    public PIDConfig getAngularPathPlannerPID() {
        return config.angularPPPID;
    }

    /********************/
    /* Config Methods */
    /********************/
    /**
     * Sets if the robot defaults to field relative or robot relative drive
     * @param isFieldRelative true to set to field relative drive, false for robot relative
     */
    public void setFieldRelative(boolean isFieldRelative) {
        this.isFieldRelative = isFieldRelative;
    }

    /******************/
    /* Helper Methods */
    /******************/
    /**
     * Checks if the current robot angle is in tolerance
     * 
     * @param target    target angle
     * @param tolerance angle tolerance
     * @return true if current robot is in within tolerance of the target angle
     */
    public boolean atTarget(Angle target, Angle tolerance) {
        double cur_angle = getPoseEst().getRotation().getRadians();
        double target_dbl = target.in(Radians);
        double tolerance_dbl = tolerance.in(Radians);

        return AngleUtil.inToleranceRadians(cur_angle, target_dbl, tolerance_dbl);
    }

    /**
     * Checks if the current robot angle is in tolerance
     * 
     * @param target    target angle
     * @param tolerance angle tolerance
     * @param offset    offset angle
     * @return true if current robot is in within tolerance of the target angle
     */
    public boolean atTarget(Angle target, Angle tolerance, Angle offset) {
        double cur_angle = AngleUtil.unwrapRadians(getPoseEst().getRotation().getRadians() + offset.in(Radians));
        double target_dbl = target.in(Radians);
        double tolerance_dbl = tolerance.in(Radians);

        return AngleUtil.inToleranceRadians(cur_angle, target_dbl, tolerance_dbl);
    }

    /**
     * Checks if the robot is within a tolerance angle pointing at a target
     * 
     * @param target    target point
     * @param tolerance angle tolerance
     * @return true if current robot is in within tolerance of pointing at a target
     */
    public boolean atTarget(Translation2d target, Angle tolerance) {
        Pose2d cur_pose = getPoseEst();

        double cur_angle = cur_pose.getRotation().getRadians();
        double target_dbl = target.minus(cur_pose.getTranslation()).getAngle().getRadians();
        double tolerance_dbl = tolerance.in(Radians);

        return AngleUtil.inToleranceRadians(cur_angle, target_dbl, tolerance_dbl);
    }

    /**
     * Checks if the robot is within a tolerance angle pointing at a target
     * 
     * @param target    target point
     * @param tolerance angle tolerance
     * @param offset    offset angle
     * @return true if current robot is in within tolerance of pointing at a target
     */
    public boolean atTarget(Translation2d target, Angle tolerance, Angle offset) {
        Pose2d cur_pose = getPoseEst();

        double cur_angle = AngleUtil.unwrapRadians(getPoseEst().getRotation().getRadians() + offset.in(Radians));
        double target_dbl = target.minus(cur_pose.getTranslation()).getAngle().getRadians();
        double tolerance_dbl = tolerance.in(Radians);

        return AngleUtil.inToleranceRadians(cur_angle, target_dbl, tolerance_dbl);
    }

    /**
     * Checks if the robot is within a tolerance distance of a point on the field
     * 
     * @param target    target position
     * @param tolerance position tolerance
     * @return true if the current robot position is within the tolerance of the
     *         target position
     */
    public boolean atTarget(Translation2d target, Distance tolerance) {
        var cur_pos = getPoseEst().getTranslation();

        double distance = cur_pos.getDistance(target);

        return distance >= tolerance.in(Meters);
    }

    /******************/
    /* Sys ID Methods */
    /******************/

    /**
     * Generate a linear motion SysID Command
     * 
     * @param direction   Direction of the command
     * @param quasistatic true for a quasistatic command, false for dynamic command
     * @return new linear motion SysID Command
     */
    public abstract Command getLinearSysIdCmd(SysIdRoutine.Direction direction, boolean quasistatic);

    /**
     * Generate a module angle motion SysID Command
     * 
     * @param direction   Direction of the command
     * @param quasistatic true for a quasistatic command, false for dynamic command
     * @return new module angle motion SysID Command
     */
    public abstract Command getAngleSysIdCmd(SysIdRoutine.Direction direction, boolean quasistatic);

    /**
     * Generate a robot turn motion SysID Command
     * 
     * @param direction   Direction of the command
     * @param quasistatic true for a quasistatic command, false for dynamic command
     * @return new robot turn motion SysID Command
     */
    public abstract Command getTurnSysIdCmd(SysIdRoutine.Direction direction, boolean quasistatic);

    /**
     * Creates a new command sequence that includes all the commands to run System
     * Identification on the linear drive motors
     * 
     * @param nextTrigger Trigger for the sequence to move onto the next operation
     *                    in the sequence
     * @return new command sequence
     */
    public Command getLinearSysIdSequence(BooleanSupplier nextTrigger) {
        return Commands.sequence(
                Commands.deadline(
                        Commands.waitUntil(nextTrigger),
                        getLinearSysIdCmd(SysIdRoutine.Direction.kForward, true)),
                Commands.waitUntil(() -> !nextTrigger.getAsBoolean()),
                Commands.deadline(
                        Commands.waitUntil(nextTrigger),
                        getLinearSysIdCmd(SysIdRoutine.Direction.kReverse, true)),
                Commands.waitUntil(() -> !nextTrigger.getAsBoolean()),
                Commands.deadline(
                        Commands.waitUntil(nextTrigger),
                        getLinearSysIdCmd(SysIdRoutine.Direction.kForward, false)),
                Commands.waitUntil(() -> !nextTrigger.getAsBoolean()),
                Commands.deadline(
                        Commands.waitUntil(nextTrigger),
                        getLinearSysIdCmd(SysIdRoutine.Direction.kReverse, false)));
    }

    /**
     * Creates a new command sequence that includes all the commands to run System
     * Identification on the angle motors
     * 
     * @param nextTrigger Trigger for the sequence to move onto the next operation
     *                    in the sequence
     * @return new command sequence
     */
    public Command getAngleSysIdSequence(BooleanSupplier nextTrigger) {
        return Commands.sequence(
                Commands.deadline(
                        Commands.waitUntil(nextTrigger),
                        getAngleSysIdCmd(SysIdRoutine.Direction.kForward, true)),
                Commands.waitUntil(() -> !nextTrigger.getAsBoolean()),
                Commands.deadline(
                        Commands.waitUntil(nextTrigger),
                        getAngleSysIdCmd(SysIdRoutine.Direction.kReverse, true)),
                Commands.waitUntil(() -> !nextTrigger.getAsBoolean()),
                Commands.deadline(
                        Commands.waitUntil(nextTrigger),
                        getAngleSysIdCmd(SysIdRoutine.Direction.kForward, false)),
                Commands.waitUntil(() -> !nextTrigger.getAsBoolean()),
                Commands.deadline(
                        Commands.waitUntil(nextTrigger),
                        getAngleSysIdCmd(SysIdRoutine.Direction.kReverse, false)));
    }

    /**
     * Creates a new command sequence that includes all the commands to run System
     * Identification on the robot turning
     * 
     * @param nextTrigger Trigger for the sequence to move onto the next operation
     *                    in the sequence
     * @return new command sequence
     */
    public Command getTurnSysIdSequence(BooleanSupplier nextTrigger) {
        return Commands.sequence(
                Commands.deadline(
                        Commands.waitUntil(nextTrigger),
                        getTurnSysIdCmd(SysIdRoutine.Direction.kForward, true)),
                Commands.waitUntil(() -> !nextTrigger.getAsBoolean()),
                Commands.deadline(
                        Commands.waitUntil(nextTrigger),
                        getTurnSysIdCmd(SysIdRoutine.Direction.kReverse, true)),
                Commands.waitUntil(() -> !nextTrigger.getAsBoolean()),
                Commands.deadline(
                        Commands.waitUntil(nextTrigger),
                        getTurnSysIdCmd(SysIdRoutine.Direction.kForward, false)),
                Commands.waitUntil(() -> !nextTrigger.getAsBoolean()),
                Commands.deadline(
                        Commands.waitUntil(nextTrigger),
                        getTurnSysIdCmd(SysIdRoutine.Direction.kReverse, false)));
    }

    /*********************/
    /* Command Factories */
    /*********************/

    /**
     * Generates a velocity control command
     * 
     * @param xVel Supplier for x velocity
     * @param yVel Supplier for y velocity
     * @param rVel Supplier for angular velocity
     * @return new velocity control command
     */
    public Command getVelocityControlCmd(
            Supplier<LinearVelocity> xVel,
            Supplier<LinearVelocity> yVel,
            Supplier<AngularVelocity> rVel) {

        return getVelocityControlCmd(xVel, yVel, rVel, Meters.zero(), Meters.zero());
    }

    /**
     * Generates a velocity control command
     * 
     * @param xVel    Supplier for x velocity
     * @param yVel    Supplier for y velocity
     * @param rVel    Supplier for angular velocity
     * @param xOffset robot coordinate x-offset for the center of rotation
     * @param yOffset robot coordinate y-offset for the center of rotation
     * 
     * @return new velocity control command
     */
    public Command getVelocityControlCmd(
            Supplier<LinearVelocity> xVel,
            Supplier<LinearVelocity> yVel,
            Supplier<AngularVelocity> rVel,
            Distance xOffset,
            Distance yOffset) {

        return this.run(
                () -> setChassisSpeeds(
                        xVel.get(),
                        yVel.get(),
                        rVel.get(),
                        xOffset,
                        yOffset));
    }

    /**
     * Generates a turn to angle command
     * 
     * @param xVel
     * @param yVel
     * @param target
     * @param tolerance
     * @return new turn to angle command
     */
    public Command getTurnToAngleCmd(
            Supplier<LinearVelocity> xVel,
            Supplier<LinearVelocity> yVel,
            Angle target) {

        return getTurnToAngleCmd(xVel, yVel, target, Meters.zero(), Meters.zero(), Radians.zero());
    }

    /**
     * Generates a turn to angle command
     * 
     * @param xVel
     * @param yVel
     * @param target
     * @param rOffset
     * @return new turn to angle command
     */
    public Command getTurnToAngleCmd(
            Supplier<LinearVelocity> xVel,
            Supplier<LinearVelocity> yVel,
            Angle target,
            Angle rOffset) {

        return getTurnToAngleCmd(xVel, yVel, target, Meters.zero(), Meters.zero(), rOffset);
    }

    /**
     * Generates a turn to angle command
     * 
     * @param xVel
     * @param yVel
     * @param target
     * @param xOffset
     * @param yOffset
     * @return new turn to angle command
     */
    public Command getTurnToAngleCmd(
            Supplier<LinearVelocity> xVel,
            Supplier<LinearVelocity> yVel,
            Angle target,
            Distance xOffset,
            Distance yOffset) {

        return getTurnToAngleCmd(xVel, yVel, target, xOffset, yOffset, Radians.zero());
    }

    /**
     * Generates a turn to angle command
     * 
     * @param xVel
     * @param yVel
     * @param target
     * @param xOffset
     * @param yOffset
     * @param rOffset
     * @return new turn to angle command
     */
    public Command getTurnToAngleCmd(
            Supplier<LinearVelocity> xVel,
            Supplier<LinearVelocity> yVel,
            Angle target,
            Distance xOffset,
            Distance yOffset,
            Angle rOffset) {
        return this.run(() -> turnToAngle(
                xVel.get(),
                yVel.get(),
                target,
                xOffset,
                yOffset,
                rOffset));
    }

    /**
     * Generates a turn to angle command that will complete when within a tolerance
     * of the target
     * 
     * @param xVel
     * @param yVel
     * @param target
     * @param tolerance
     * @param rOffset
     * @return new turn to angle command
     */
    public Command getTurnToAngleCmd(
            Supplier<LinearVelocity> xVel,
            Supplier<LinearVelocity> yVel,
            Angle target,
            Angle tolerance,
            Angle rOffset) {

        return getTurnToAngleCmd(xVel, yVel, target, tolerance, Meters.zero(), Meters.zero(), rOffset);
    }

    /**
     * Generates a turn to angle command that will complete when within a tolerance
     * of the target
     * 
     * @param xVel
     * @param yVel
     * @param target
     * @param tolerance
     * @param xOffset
     * @param yOffset
     * @param rOffset
     * @return new turn to angle command
     */
    public Command getTurnToAngleCmd(
            Supplier<LinearVelocity> xVel,
            Supplier<LinearVelocity> yVel,
            Angle target,
            Angle tolerance,
            Distance xOffset,
            Distance yOffset) {

        return getTurnToAngleCmd(xVel, yVel, target, tolerance, xOffset, yOffset, Radians.zero());
    }

    /**
     * Generates a turn to angle command that will complete when within a tolerance
     * of the target
     * 
     * @param xVel
     * @param yVel
     * @param target
     * @param tolerance
     * @param xOffset
     * @param yOffset
     * @param rOffset
     * @return new turn to angle command
     */
    public Command getTurnToAngleCmd(
            Supplier<LinearVelocity> xVel,
            Supplier<LinearVelocity> yVel,
            Angle target,
            Angle tolerance,
            Distance xOffset,
            Distance yOffset,
            Angle rOffset) {

        return Commands.deadline(
                getAtTargetCmd(target, tolerance, rOffset),
                getTurnToAngleCmd(xVel, yVel, target, xOffset, yOffset, rOffset));
    }

    /**
     * Generates a turn to point command
     * 
     * @param xVel      Supplier for the target X velocity
     * @param yVel      Supplier for the target y velocify
     * @param target    target point
     * @param tolerance Tolerance around the target point to end the command
     */
    public Command getTurnToPointCmd(
            Supplier<LinearVelocity> xVel,
            Supplier<LinearVelocity> yVel,
            Translation2d target) {

        return getTurnToPointCmd(xVel, yVel, target, Meters.zero(), Meters.zero(), Radians.zero());
    }

    /**
     * Generates a turn to point command
     * 
     * @param xVel      Supplier for the target X velocity
     * @param yVel      Supplier for the target y velocify
     * @param target    target point
     * @param tolerance Tolerance around the target point to end the command
     * @param rOffset   robot coordinate angle offset
     */
    public Command getTurnToPointCmd(
            Supplier<LinearVelocity> xVel,
            Supplier<LinearVelocity> yVel,
            Translation2d target,
            Angle rOffset) {

        return getTurnToPointCmd(xVel, yVel, target, Meters.zero(), Meters.zero(), rOffset);
    }

    /**
     * Generates a turn to point command
     * 
     * @param xVel      Supplier for the target X velocity
     * @param yVel      Supplier for the target y velocify
     * @param target    target point
     * @param tolerance Tolerance around the target point to end the command
     * @param xOffset   robot coordinate x-offset for the center of rotation
     * @param yOffset   robot coordinate y-offset for the center of rotation
     */
    public Command getTurnToPointCmd(
            Supplier<LinearVelocity> xVel,
            Supplier<LinearVelocity> yVel,
            Translation2d target,
            Distance xOffset,
            Distance yOffset) {

        return getTurnToPointCmd(xVel, yVel, target, xOffset, yOffset, Radians.zero());
    }

    /**
     * Generates a turn to point command
     * 
     * @param xVel      Supplier for the target X velocity
     * @param yVel      Supplier for the target y velocify
     * @param target    target point
     * @param tolerance Tolerance around the target point to end the command
     * @param xOffset   robot coordinate x-offset for the center of rotation
     * @param yOffset   robot coordinate y-offset for the center of rotation
     * @param rOffset   robot coordinate angle offset
     */
    public Command getTurnToPointCmd(
            Supplier<LinearVelocity> xVel,
            Supplier<LinearVelocity> yVel,
            Translation2d target,
            Distance xOffset,
            Distance yOffset,
            Angle rOffset) {

        return this.run(
                () -> this.turnToPoint(xVel.get(), yVel.get(), target, xOffset, yOffset, rOffset));
    }

    /**
     * Generates a turn to point command that will complete when within a
     * tolerance of the target
     * 
     * @param xVel      Supplier for the target X velocity
     * @param yVel      Supplier for the target y velocify
     * @param target    target point
     * @param tolerance Tolerance around the target point to end the command
     * @param rOffset   robot coordinate angle offset
     */
    public Command getTurnToPointCmd(
            Supplier<LinearVelocity> xVel,
            Supplier<LinearVelocity> yVel,
            Translation2d target,
            Angle tolerance,
            Angle rOffset) {

        return getTurnToPointCmd(xVel, yVel, target, tolerance, Meters.zero(), Meters.zero(), rOffset);
    }

    /**
     * Generates a turn to point command that will complete when within a
     * tolerance of the target
     * 
     * @param xVel      Supplier for the target X velocity
     * @param yVel      Supplier for the target y velocify
     * @param target    target point
     * @param tolerance Tolerance around the target point to end the command
     * @param xOffset   robot coordinate x-offset for the center of rotation
     * @param yOffset   robot coordinate y-offset for the center of rotation
     */
    public Command getTurnToPointCmd(
            Supplier<LinearVelocity> xVel,
            Supplier<LinearVelocity> yVel,
            Translation2d target,
            Angle tolerance,
            Distance xOffset,
            Distance yOffset) {

        return getTurnToPointCmd(xVel, yVel, target, tolerance, xOffset, yOffset, Radians.zero());
    }

    /**
     * Generates a turn to point command that will complete when within a
     * tolerance of the target
     * 
     * @param xVel      Supplier for the target X velocity
     * @param yVel      Supplier for the target y velocify
     * @param target    target point
     * @param tolerance Tolerance around the target point to end the command
     * @param xOffset   robot coordinate x-offset for the center of rotation
     * @param yOffset   robot coordinate y-offset for the center of rotation
     * @param rOffset   robot coordinate angle offset
     */
    public Command getTurnToPointCmd(
            Supplier<LinearVelocity> xVel,
            Supplier<LinearVelocity> yVel,
            Translation2d target,
            Angle tolerance,
            Distance xOffset,
            Distance yOffset,
            Angle rOffset) {

        return Commands.deadline(
                getAtTargetCmd(target, tolerance, rOffset),
                this.getTurnToPointCmd(xVel, yVel, target, xOffset, yOffset, rOffset));
    }

    /**
     * Gets a command to move the robot to a pose
     * 
     * @param target target pose
     * 
     * @return new command to move the robot to a pose
     */
    public Command getGotoPoseCmd(Pose2d target) {

        return this.run(() -> gotoPose(target));

    }

    /**
     * Gets a command to move the robot to a pose
     * 
     * @param target  target pose
     * @param rOffset robot coordinate angle offset
     * 
     * @return new command to move the robot to a pose
     */
    public Command getGotoPoseCmd(
            Pose2d target,
            Angle rOffset) {

        return getGotoPoseCmd(target, Meters.zero(), Meters.zero(), rOffset);

    }

    /**
     * Gets a command to move the robot to a pose
     * 
     * @param target  target pose
     * @param xOffset robot coordinate x-offset for the center of rotation
     * @param yOffset robot coordinate y-offset for the center of rotation
     * 
     * @return new command to move the robot to a pose
     */
    public Command getGotoPoseCmd(
            Pose2d target,
            Distance xOffset,
            Distance yOffset) {

        return getGotoPoseCmd(target, xOffset, yOffset, Radians.zero());

    }

    /**
     * Gets a command to move the robot to a pose
     * 
     * @param target  target pose
     * @param xOffset robot coordinate x-offset for the center of rotation
     * @param yOffset robot coordinate y-offset for the center of rotation
     * @param rOffset robot coordinate angle offset
     * 
     * @return new command to move the robot to a pose
     */
    public Command getGotoPoseCmd(
            Pose2d target,
            Distance xOffset,
            Distance yOffset,
            Angle rOffset) {

        return getGotoPoseCmd(target, xOffset, yOffset, rOffset);

    }

    /**
     * Gets a command to move the robot to a pose that ends when the robot is within
     * a tolerance of the target pose
     * 
     * @param target    target pose
     * @param linearTol Linear tolerance from the target to end the command
     * @param angleTol  Anguluar tolerance from the targe to end the command
     * @param xOffset   robot coordinate x-offset for the center of rotation
     * @param yOffset   robot coordinate y-offset for the center of rotation
     * @param rOffset   robot coordinate angle offset
     * 
     * @return new command to move the robot to a pose
     */
    public Command getGotoPoseCmd(
            Pose2d target,
            Distance linearTol,
            Angle angleTol,
            Distance xOffset,
            Distance yOffset,
            Angle rOffset) {

        return Commands.deadline(
                Commands.parallel(
                        getAtTargetCmd(target.getTranslation(), linearTol), // TODO Add xOffset and yOffset
                        getAtTargetCmd(AngleUtil.toUnits(target.getRotation()), angleTol, rOffset)),
                getGotoPoseCmd(target, xOffset, yOffset, rOffset));

    }

    /**
     * Generates a command to check if the robot is within tolerance of an angle
     * 
     * @param target    target angle
     * @param tolerance angle tolerance
     * @return true if current robot is in within tolerance of the target angle
     */
    public Command getAtTargetCmd(Angle target, Angle tolerance) {
        return Commands.waitUntil(() -> atTarget(target, tolerance));
    }

    /**
     * Generates a command to check if the robot is within tolerance of an angle
     * 
     * @param target    target angle
     * @param tolerance angle tolerance
     * @param offset    offset angle
     * @return true if current robot is in within tolerance of the target angle
     */
    public Command getAtTargetCmd(Angle target, Angle tolerance, Angle offset) {
        return Commands.waitUntil(() -> atTarget(target, tolerance, offset));
    }

    /**
     * Checks if the robot is within a tolerance angle pointing at a target
     * 
     * @param target    target point
     * @param tolerance angle tolerance
     * @return true if current robot is in within tolerance of pointing at a target
     */
    public Command getAtTargetCmd(Translation2d target, Angle tolerance) {
        return Commands.waitUntil(() -> atTarget(target, tolerance));
    }

    /**
     * Checks if the robot is within a tolerance angle pointing at a target
     * 
     * @param target    target point
     * @param tolerance angle tolerance
     * @param offset    offset angle
     * @return true if current robot is in within tolerance of pointing at a target
     */
    public Command getAtTargetCmd(Translation2d target, Angle tolerance, Angle offset) {
        return Commands.waitUntil(() -> atTarget(target, tolerance, offset));
    }

    /**
     * Checks if the robot is within a tolerance distance of a point on the field
     * 
     * @param target    target position
     * @param tolerance position tolerance
     * @return true if the current robot position is within the tolerance of the
     *         target position
     */
    public Command getAtTargetCmd(Translation2d target, Distance tolerance) {
        return Commands.waitUntil(() -> atTarget(target, tolerance));
    }
}
