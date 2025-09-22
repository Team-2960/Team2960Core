package frc.lib2960.telemetry;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Makes a sendable out of a Chassis Speeds
 */
public class SendablePose2d implements Sendable {
    private final Pose2d pose;
    private final MutDistance x = Meters.mutable(0);
    private final MutDistance y = Meters.mutable(0);
    private final MutAngle r = Radians.mutable(0);

    /**
     * Constructor
     * 
     * @param speeds Chassis speeds to make sendable
     */
    public SendablePose2d(Pose2d pose) {
        this.pose = pose;
    }

    /**
     * Implements sendable initialization
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ChassisSpeeds");
        builder.addStringProperty("X", () -> x.mut_replace(pose.getX(), Meters).toShortString(), null);
        builder.addStringProperty("Y", () -> y.mut_replace(pose.getY(), Meters).toShortString(), null);
        builder.addStringProperty("R", () -> r.mut_replace(pose.getRotation().getRadians(), Radians).toShortString(),
                null);

    }
}
