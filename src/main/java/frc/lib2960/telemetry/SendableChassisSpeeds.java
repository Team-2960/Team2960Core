package frc.lib2960.telemetry;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Makes a sendable out of a Chassis Speeds
 */
public class SendableChassisSpeeds implements Sendable {
    private final ChassisSpeeds speeds;
    private final MutLinearVelocity xVel = MetersPerSecond.mutable(0);
    private final MutLinearVelocity yVel = MetersPerSecond.mutable(0);
    private final MutAngularVelocity rVel = RadiansPerSecond.mutable(0);

    /**
     * Constructor
     * 
     * @param speeds Chassis speeds to make sendable
     */
    public SendableChassisSpeeds(ChassisSpeeds speeds) {
        this.speeds = speeds;
    }

    /**
     * Implements sendable initialization
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ChassisSpeeds");
        builder.addStringProperty("X",
                () -> xVel.mut_replace(speeds.vxMetersPerSecond, MetersPerSecond).toShortString(), null);
        builder.addStringProperty("Y",
                () -> yVel.mut_replace(speeds.vyMetersPerSecond, MetersPerSecond).toShortString(), null);
        builder.addStringProperty("R",
                () -> rVel.mut_replace(speeds.vxMetersPerSecond, RadiansPerSecond).toShortString(), null);

    }
}
