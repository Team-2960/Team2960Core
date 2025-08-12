package frc.lib2960.settings;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngularAcceleration;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutLinearAcceleration;
import edu.wpi.first.units.measure.MutLinearVelocity;

public class SwerveDriveBaseSettings {
    public final MutLinearVelocity maxLinearVel;
    public final MutLinearAcceleration maxLinearAccel;
    public final MutAngularVelocity maxAngularVel;
    public final MutAngularAcceleration maxAngularAccel;

    public final SwerveModuleCommonSettings commonModuleSettings;

    public final SwerveModuleBaseSettings[] moduleSettings;

    public SwerveDriveBaseSettings(
            LinearVelocity maxLinearVel,
            LinearAcceleration maxLinearAccel,
            AngularVelocity maxAngularVel,
            AngularAcceleration maxAngularAccel,
            SwerveModuleCommonSettings commonModuleSettings,
            SwerveModuleBaseSettings... moduleSettings) {

        this.maxLinearVel = maxLinearVel.mutableCopy();
        this.maxLinearAccel = maxLinearAccel.mutableCopy();
        this.maxAngularVel = maxAngularVel.mutableCopy();
        this.maxAngularAccel = maxAngularAccel.mutableCopy();

        this.commonModuleSettings = commonModuleSettings;

        this.moduleSettings = moduleSettings;
    }
}
