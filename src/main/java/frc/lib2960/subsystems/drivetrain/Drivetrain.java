package frc.lib2960.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Drivetrain extends Subsystem {
    public Pose2d getPoseEst();

    public default ChassisSpeeds getFieldRelativeSpeeds(){
        var pose = getPoseEst();
        var speeds = getRobotRelativeSpeeds();
        return ChassisSpeeds.fromRobotRelativeSpeeds(speeds, pose.getRotation());
    }

    public ChassisSpeeds getRobotRelativeSpeeds();

    public void resetPose(Pose2d pose);

    public void addVisionMeasurement(Pose2d pose, Time timestamp, Vector<N3> std);

    public default void setPathPlannerSpeeds(ChassisSpeeds speeds) {
        setChassisSpeeds(speeds, false, Meters.zero(), Meters.zero());
    }
    
    public void setChassisSpeeds(ChassisSpeeds speeds, boolean isFieldRelative, Distance xOffset, Distance yOffset);
}
