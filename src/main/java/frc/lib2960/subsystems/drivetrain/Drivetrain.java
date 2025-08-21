package frc.lib2960.subsystems.drivetrain;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public interface Drivetrain {
    public Pose2d getPoseEst();

    public ChassisSpeeds getChassisSpeeds();

    public void resetPose(Pose2d pose);

    public void addVisionMeasurement(Pose2d pose, Time timestamp, Vector<N3> std);

    public void setChassisSpeeds(ChassisSpeeds speeds, boolean isFieldRelative, Distance xOffset, Distance yOffset);
}
