package frc.lib2960.subsystem.drivetrain;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Subsystem;import frc.lib2960.config.basic.RobotFeature;

public interface Drivetrain extends Subsystem {
    /**
     * Gets the current estimated pose of the robot
     * 
     * @return current estimated pose fo the robot
     */
    public Pose2d getPoseEst();

    /**
     * Gets the chassis speeds relative to the field's frame of reference
     * 
     * @return ChassisSpeeds relative to the field's frame of reference
     */
    public default ChassisSpeeds getFieldRelativeSpeeds() {
        var pose = getPoseEst();
        var speeds = getRobotRelativeSpeeds();
        return ChassisSpeeds.fromRobotRelativeSpeeds(speeds, pose.getRotation());
    }

    /**
     * Gets the chassis speeds relative to the robot's frame of reference
     * 
     * @return ChassisSpeeds relative to the robot's frame of reference
     */
    public ChassisSpeeds getRobotRelativeSpeeds();

    /**
     * Sets the pose of the robot to a pre-defined position
     * 
     * @param pose new pose for the robot
     */
    public void resetPose(Pose2d pose);

    /**
     * Adds a vision measurement to the pose estimator
     * 
     * @param pose      estimated pose from vision
     * @param timestamp vision measurement timestamp
     * @param std       standard deviation vector for the estimated pose
     */
    public void addVisionMeasurement(Pose2d pose, Time timestamp, Vector<N3> std);

    /**
     * Sets chassis speeds for path planner. isFieldRelative is set to False, and
     * center of rotation offset is set to zero.
     * 
     * @param speeds
     */
    public default void setPathPlannerSpeeds(ChassisSpeeds speeds) {
        setChassisSpeeds(speeds, false, RobotFeature.origin);
    }

    /**
     * Sets the chassis speeds for the drivetrain
     * 
     * @param speeds          Chassis speeds for the drivetrain
     * @param isFieldRelative Sets if the chassis speeds should be field relative or
     *                        robot relative
     * @param feature Robot feature offset
     */
    public void setChassisSpeeds(ChassisSpeeds speeds, boolean isFieldRelative, RobotFeature feature);
}
