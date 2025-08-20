package frc.lib2960.subsystems;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.lib2960.settings.CTRESwerveDriveSettings;
import frc.lib2960.settings.SwerveModuleBaseSettings;
import frc.lib2960.settings.SwerveModuleCommonSettings;

public class CTRESwerveDrive extends SwerveDriveBase {


    public CTRESwerveDrive(CTRESwerveDriveSettings settings, SwerveModuleCommonSettings commonSettings, SwerveModuleBaseSettings... moduleSettings) {
        super(settings);
    }

    @Override
    public void setChassisSpeeds(ChassisSpeeds speeds, boolean isFieldRelative, Distance xOffset, Distance yOffset) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setChassisSpeeds'");
    }

    @Override
    public Pose2d getPoseEst() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPoseEst'");
    }

    @Override
    public ChassisSpeeds getChassisSpeeds() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getChassisSpeeds'");
    }

    @Override
    public void resetPose(Pose2d pose) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetPose'");
    }

    @Override
    public void addVisionMeasurment(Pose2d pose, Time timestamp, Vector<N3> std) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'addVisionMeasurment'");
    }
    
}
