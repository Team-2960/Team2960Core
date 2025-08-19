package frc.lib2960.settings;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;

public class SwerveModuleCommonSettings {
    public final Distance wheelDiameter;
    public final Dimensionless driveRatio;
    public final Dimensionless angleRatio;
    public final Dimensionless coupleRatio;

    public final PIDSettings drivePID;
    public final FFSettings driveFF;

    public final PIDSettings anglePID;
    public final FFSettings angleFF;

    public final AngularVelocity maxAngleVel;
    public final AngularAcceleration maxAngleAccel;

    public SwerveModuleCommonSettings(
            Distance wheelDiameter,
            Dimensionless driveRatio,
            Dimensionless angleRatio,
            Dimensionless coupleRatio,
            PIDSettings drivePID,
            FFSettings driveFF,
            PIDSettings anglePID,
            FFSettings angleFF, 
            AngularVelocity maxAngleVel, 
            AngularAcceleration maxAngleAccel
            ) {
        
        this.wheelDiameter = wheelDiameter;
        this.driveRatio = driveRatio;
        this.angleRatio = angleRatio;
        this.coupleRatio = coupleRatio;
        this.drivePID = drivePID;
        this.driveFF = driveFF;
        this.anglePID = anglePID;
        this.angleFF = angleFF;
        this.maxAngleVel = maxAngleVel;
        this.maxAngleAccel = maxAngleAccel;

    }
}
