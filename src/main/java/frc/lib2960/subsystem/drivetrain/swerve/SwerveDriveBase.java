package frc.lib2960.subsystem.drivetrain.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib2960.controller.AngularController;
import frc.lib2960.controller.LinearController;
import frc.lib2960.subsystem.drivetrain.HolonomicDrivetrain;
import frc.lib2960.config.controller.PIDConfig;
import frc.lib2960.helper.AngleUtil;
import frc.lib2960.config.basic.RobotFeature;

/**
 * Defines core capabilities of a swerve drive
 */
public abstract class SwerveDriveBase extends SubsystemBase implements HolonomicDrivetrain {
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

    protected final ShuffleboardTab tab;
    protected final Field2d field = new Field2d();

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
        this.angleCtrl = new AngularController(config.angularCtrlConfig);

        tab = Shuffleboard.getTab(config.uiTabName);

        tab.add("Subsystem", this);
        linearCtrl.addToUI("Linear Controller", tab);
        angleCtrl.addToUI("Angular Controller", tab);
        tab.add("Swerve Drive", getSwerveSendable());
        tab.add("Status", getStatusSendable());
        tab.add("Field", field);

        // TODO Enable telemetry for when methods are overloaded

    }

    /*********************/
    /* Telemetry Methods */
    /*********************/

    /**
     * Generates a sendable to for showing the current status of the swerve drive
     * 
     * @return sendable to for showing the current status of the swerve drive
     */
    public abstract Sendable getSwerveSendable();

    /**
     * Generates a sendable with swerve drive status
     * 
     * @return
     */
    public Sendable getStatusSendable() {
        DistanceUnit linPosUnit = config.linPosUnit;
        TimeUnit linTimeUnit = config.linTimeUnit;
        LinearVelocityUnit linVelUnit = linPosUnit.per(linTimeUnit);
        String linPosUnitStr = "(" + linPosUnit.symbol() + ")";
        String linVelUnitStr = "(" + linPosUnit.symbol() + " per " + linTimeUnit.symbol() + ")";
        
        AngleUnit angPosUnit = config.angPosUnit;
        TimeUnit angTimeUnit = config.angTimeUnit;
        AngularVelocityUnit angVelUnit = angPosUnit.per(angTimeUnit);
        String angPosUnitStr = "(" + angPosUnit.symbol() + ")";
        String angVelUnitStr = "(" + angPosUnit.symbol() + " per " + angTimeUnit.symbol() + ")";
        

        return new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("Position Error " + linPosUnitStr, () -> posErrorCalc.in(linPosUnit), null);
                builder.addDoubleProperty("Target Vel Mag " + linVelUnitStr, () -> velMagCalc.in(linVelUnit), null);
                builder.addDoubleProperty("Target X Vel " + linVelUnitStr, () -> xVelCalc.in(linVelUnit), null);
                builder.addDoubleProperty("Target Y Vel " + linVelUnitStr, () -> yVelCalc.in(linVelUnit), null);;
                builder.addDoubleProperty("Target Angle Pos " + angPosUnitStr, () -> angleTarget.in(angPosUnit), null);
                builder.addDoubleProperty("Target Angle Vel " + angVelUnitStr, () -> angleVelCalc.in(angVelUnit), null);

            }
        };
    }

    @Override
    public void periodic() {
        // Update Telemetry
        field.setRobotPose(getPoseEst());
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
        gotoPose(target, RobotFeature.origin);
    }

    /**
     * Updates the swerve drive base to the given target pose
     * 
     * @param target  target pose
     * @param feature Robot feature offset
     */
    public void gotoPose(Pose2d target, RobotFeature feature) {
        calcVelToPosition(target.getTranslation(), xVelCalc, yVelCalc);
        angleTarget.mut_replace(target.getRotation().getRadians(), Radians);
        turnToAngle(xVelCalc, yVelCalc, angleTarget, feature);
    }

    /**
     * Updates the swerve drive base to point towarwd a given target
     * 
     * @param xVel   Target x velocity
     * @param yVel   Target y velocity
     * @param target Target point
     */
    public final void turnToPoint(LinearVelocity xVel, LinearVelocity yVel, Translation2d target) {
        turnToPoint(xVel, yVel, target, RobotFeature.origin);
    }

    /**
     * Updates the swerve drive base to point towarwd a given target
     * 
     * @param xVel    Target x velocity
     * @param yVel    Target y velocity
     * @param target  Target point
     * @param feature Robot feature offset
     */
    public void turnToPoint(LinearVelocity xVel, LinearVelocity yVel, Translation2d target, RobotFeature feature) {
        calcAngleToPoint(target, angleTarget);
        turnToAngle(xVel, yVel, angleTarget, feature);
    }

    /**
     * Updates the swerve drive base to turn to a given angle
     * 
     * @param xVel   Target x velocity
     * @param yVel   Target y velocity
     * @param target Target angle
     */
    public final void turnToAngle(LinearVelocity xVel, LinearVelocity yVel, Angle target) {
        turnToAngle(xVel, yVel, target, RobotFeature.origin);
    }

    /**
     * Updates the swerve drive base to turn to a given angle
     * 
     * @param xVel    Target x velocity
     * @param yVel    Target y velocity
     * @param target  Target angle
     * @param feature Robot feature offset
     */
    public void turnToAngle(LinearVelocity xVel, LinearVelocity yVel, Angle target, RobotFeature feature) {

        calcVelToAngle(target, angleVelCalc);
        targetSpeeds.vxMetersPerSecond = xVel.in(MetersPerSecond);
        targetSpeeds.vyMetersPerSecond = yVel.in(MetersPerSecond);
        targetSpeeds.omegaRadiansPerSecond = angleVelCalc.in(RadiansPerSecond);

        setChassisSpeeds(targetSpeeds, feature);
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
        setChassisSpeeds(xVel, yVel, rVel, this.isFieldRelative, RobotFeature.origin);
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
        setChassisSpeeds(xVel, yVel, rVel, isFieldRelative, RobotFeature.origin);
    }

    /**
     * Sets the target chassis speeds of the robot. isFieldRelative uses the value
     * set in setFieldRelative method.
     * 
     * @param xVel    target x velocity
     * @param yVel    target y velocity
     * @param rVel    target rotation velocity
     * @param feature Robot feature offset
     */
    public final void setChassisSpeeds(LinearVelocity xVel, LinearVelocity yVel, AngularVelocity rVel,
            RobotFeature feature) {
        setChassisSpeeds(xVel, yVel, rVel, this.isFieldRelative, feature);
    }

    /**
     * Sets the target chassis speeds of the robot. isFieldRelative uses the value
     * set in setFieldRelative method.
     * 
     * @param xVel    target x velocity
     * @param yVel    target y velocity
     * @param rVel    target rotation velocity
     * @param feature Robot feature offset
     */
    public final void setChassisSpeeds(LinearVelocity xVel, LinearVelocity yVel, AngularVelocity rVel,
            boolean isFieldRelative, RobotFeature feature) {
        setChassisSpeeds(
                new ChassisSpeeds(
                        xVel.in(MetersPerSecond),
                        yVel.in(MetersPerSecond),
                        rVel.in(RadiansPerSecond)),
                this.isFieldRelative,
                feature);
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
        setChassisSpeeds(speeds, isFieldRelative, RobotFeature.origin);
    }

    /**
     * Sets the target chassis speeds of the robot. isFieldRelative uses the value
     * set in setFieldRelative method.
     * 
     * @param speeds  target chassis speeds
     * @param feature Robot feature offset
     */
    public final void setChassisSpeeds(ChassisSpeeds speeds, RobotFeature feature) {
        setChassisSpeeds(speeds, this.isFieldRelative, feature);
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
     * 
     * @param isFieldRelative true to set to field relative drive, false for robot
     *                        relative
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
     * Generates a velocity command
     * 
     * @param xVel x velocity
     * @param yVel y velocity
     * @param rVel angular velocity
     * @return new velocity command
     */
    public Command getVelocityCmd(
            LinearVelocity xVel,
            LinearVelocity yVel,
            AngularVelocity rVel) {

        Command cmd = getVelocityCmd(xVel, yVel, rVel, RobotFeature.origin);

        return cmd;
    }

    /**
     * Generates a velocity command
     * 
     * @param xVel    x velocity
     * @param yVel    y velocity
     * @param rVel    angular velocity
     * @param feature Robot feature offset
     * 
     * @return new velocity command
     */
    public Command getVelocityCmd(
            LinearVelocity xVel,
            LinearVelocity yVel,
            AngularVelocity rVel,
            RobotFeature feature) {

        Command cmd = this.run(() -> setChassisSpeeds(xVel, yVel, rVel, feature));

        cmd.setName(String.format("VelocityCmd F:(%s)", feature.name));

        return cmd;
    }

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

        return getVelocityControlCmd(xVel, yVel, rVel, RobotFeature.origin);
    }

    /**
     * Generates a velocity control command
     * 
     * @param xVel    Supplier for x velocity
     * @param yVel    Supplier for y velocity
     * @param rVel    Supplier for angular velocity
     * @param feature Robot feature offset
     * 
     * @return new velocity control command
     */
    public Command getVelocityControlCmd(
            Supplier<LinearVelocity> xVel,
            Supplier<LinearVelocity> yVel,
            Supplier<AngularVelocity> rVel,
            RobotFeature feature) {

        Command cmd = this.run(() -> setChassisSpeeds(xVel.get(), yVel.get(), rVel.get(), feature));

        cmd.setName(String.format("VelocityCtrlCmd F:(%s)", feature.name));

        return cmd;
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

        return getTurnToAngleCmd(xVel, yVel, target, RobotFeature.origin);
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
            RobotFeature feature) {

        Command cmd = this.run(() -> turnToAngle(
                xVel.get(),
                yVel.get(),
                target,
                feature));

        cmd.setName(String.format("TurnToAngleCmd T: %.0f\u00B0C F:(%s)",
                target.in(Degrees), feature.name));

        return cmd;
    }

    /**
     * Generates a turn to angle command that will complete when within a tolerance
     * of the target
     * 
     * @param xVel
     * @param yVel
     * @param target
     * @param tolerance
     * @param feature   Robot feature offset
     * @return new turn to angle command
     */
    public Command getTurnToAngleCmd(
            Supplier<LinearVelocity> xVel,
            Supplier<LinearVelocity> yVel,
            Angle target,
            Angle tolerance,
            RobotFeature feature) {

        Command cmd = Commands.deadline(
                getAtTargetCmd(target, tolerance, feature.r),
                getTurnToAngleCmd(xVel, yVel, target, feature));

        cmd.setName(String.format("TurnToAngleCmd and End T: %.0f\u00B0C F:(%s)", target.in(Degrees), feature));

        return cmd;
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

        return getTurnToPointCmd(xVel, yVel, target, RobotFeature.origin);
    }

    /**
     * Generates a turn to point command
     * 
     * @param xVel      Supplier for the target X velocity
     * @param yVel      Supplier for the target y velocify
     * @param target    target point
     * @param tolerance Tolerance around the target point to end the command
     * @param feature   Robot feature offset
     */
    public Command getTurnToPointCmd(
            Supplier<LinearVelocity> xVel,
            Supplier<LinearVelocity> yVel,
            Translation2d target,
            RobotFeature feature) {

        Command cmd = this.run(
                () -> this.turnToPoint(xVel.get(), yVel.get(), target, feature));

        cmd.setName(String.format("TurnToPointCmd T: (%.2f m, %.2f m) F:(%s)", target.getX(), target.getY(), feature));

        return cmd;
    }

    /**
     * Generates a turn to point command that will complete when within a
     * tolerance of the target
     * 
     * @param xVel      Supplier for the target X velocity
     * @param yVel      Supplier for the target y velocify
     * @param target    target point
     * @param tolerance Tolerance around the target point to end the command
     * @param feature   Robot feature offset
     */
    public Command getTurnToPointCmd(
            Supplier<LinearVelocity> xVel,
            Supplier<LinearVelocity> yVel,
            Translation2d target,
            Angle tolerance,
            RobotFeature feature) {

        Command cmd = Commands.deadline(
                getAtTargetCmd(target, tolerance, feature.r),
                this.getTurnToPointCmd(xVel, yVel, target, feature));

        cmd.setName(String.format("TurnToPointCmd and End T: (%.2f m, %.2f m) F:(%s)",
                target.getX(), target.getY(), feature.name));

        return cmd;
    }

    /**
     * Gets a command to move the robot to a pose
     * 
     * @param target target pose
     * 
     * @return new command to move the robot to a pose
     */
    public Command getGotoPoseCmd(Pose2d target) {
        return getGotoPoseCmd(target, RobotFeature.origin);
    }

    /**
     * Gets a command to move the robot to a pose
     * 
     * @param target  target pose
     * @param feature   Robot feature offset
     * 
     * @return new command to move the robot to a pose
     */
    public Command getGotoPoseCmd(
            Pose2d target,
            RobotFeature feature) {

        Command cmd = this.run(() -> gotoPose(target, feature));

        cmd.setName(String.format("GotoPoseCmd T:(%.2f m, %.2f m, %.0f\u00B0C) F:(%s)",
                target.getX(), target.getY(), target.getRotation().getDegrees(), feature.name));

        return cmd;
    }

    /**
     * Gets a command to move the robot to a pose that ends when the robot is within
     * a tolerance of the target pose
     * 
     * @param target    target pose
     * @param linearTol Linear tolerance from the target to end the command
     * @param angleTol  Angular tolerance from the targe to end the command
     * @param feature   Robot feature offset
     * 
     * @return new command to move the robot to a pose
     */
    public Command getGotoPoseCmd(
            Pose2d target,
            Distance linearTol,
            Angle angleTol,
            RobotFeature feature) {

        Command cmd = Commands.deadline(
                Commands.parallel(
                        getAtTargetCmd(target.getTranslation(), linearTol), // TODO Add xOffset and yOffset
                        getAtTargetCmd(AngleUtil.toUnits(target.getRotation()), angleTol, feature.r)),
                getGotoPoseCmd(target, feature));

        cmd.setName(
                String.format("GotoPoseCmd and End T:(%.2f m, %.2f m, %.0f\u00B0C) F:(%s)",
                        target.getX(), target.getY(), target.getRotation().getDegrees(), feature.name));

        return cmd;
    }

    /**
     * Generates a command to check if the robot is within tolerance of an angle
     * 
     * @param target    target angle
     * @param tolerance angle tolerance
     * @return true if current robot is in within tolerance of the target angle
     */
    public Command getAtTargetCmd(Angle target, Angle tolerance) {
        Command cmd = Commands.waitUntil(() -> atTarget(target, tolerance));
        cmd.setName(String.format("AtTargetCmd TurnToAngle T: %.2f\u00B0C", target.in(Degrees)));
        return cmd;
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
        Command cmd = Commands.waitUntil(() -> atTarget(target, tolerance, offset));
        cmd.setName(String.format("AtTargetCmd TurnToAngle T: %.2f\u00B0C O: %.2f\u00B0C",
                target.in(Degrees), offset.in(Degrees)));
        return cmd;
    }

    /**
     * Checks if the robot is within a tolerance angle pointing at a target
     * 
     * @param target    target point
     * @param tolerance angle tolerance
     * @return true if current robot is in within tolerance of pointing at a target
     */
    public Command getAtTargetCmd(Translation2d target, Angle tolerance) {
        Command cmd = Commands.waitUntil(() -> atTarget(target, tolerance));
        cmd.setName(String.format("AtTargetCmd LookAtPoint T:(%.02f m, %.02f m)",
                target.getX(), target.getY()));
        return cmd;
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
        Command cmd = Commands.waitUntil(() -> atTarget(target, tolerance, offset));
        cmd.setName(String.format("AtTargetCmd LookAtPoint T:(%.02f m, %.02f m) O: %.2f\u00B0C",
                target.getX(), target.getY(), offset.in(Degrees)));
        return cmd;
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
        Command cmd = Commands.waitUntil(() -> atTarget(target, tolerance));
        cmd.setName(String.format("AtTargetCmd GotoPose T: T:(%.02f m, %.02f m) ",
                target.getX(), target.getY()));
        return cmd;
    }
}
