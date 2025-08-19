package frc.lib2960.subsystems;


import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2960.settings.SwerveModuleBaseSettings;
import frc.lib2960.settings.SwerveModuleCommonSettings;
import frc.robot.Constants;

public abstract class SwerveModuleBase extends SubsystemBase{
    /**********************/
    /* Settings Variables */
    /**********************/
    protected final SwerveModuleCommonSettings commonSettings;
    protected final SwerveModuleBaseSettings settings;

    /*************************/
    /* Calculation Variables */
    /*************************/
    private final MutVoltage driveVoltCalc = Volts.mutable(0);
    private final MutVoltage angleVoltCalc = Volts.mutable(0);

    private final PIDController drivePID;
    private final SimpleMotorFeedforward driveFF;
    private final PIDController anglePID;
    private final SimpleMotorFeedforward angleFF;

    private final AngularVelocity maxAngleVelDelta;
    
    /****************************/
    /* Driver Station Variables */
    /****************************/

    /****************/
    /* Constructors */
    /****************/
    public SwerveModuleBase(SwerveModuleCommonSettings commonSettings, SwerveModuleBaseSettings settings) {
        this.commonSettings = commonSettings;
        this.settings = settings;

        this.drivePID = commonSettings.drivePID.getPIDController();
        this.driveFF = commonSettings.driveFF.getSimpleMotorFF();
        this.anglePID = commonSettings.anglePID.getPIDController();
        this.angleFF = commonSettings.angleFF.getSimpleMotorFF();

        maxAngleVelDelta = commonSettings.maxAngleAccel.times(Constants.commonSettings.updatePeriod);
    }


    /*******************/
    /* Control Methods */
    /*******************/
    public void setState(SwerveModuleState state) {
        // TODO implement
    }

    public abstract void setDriveVolt(Voltage volts);

    public abstract void setAngleVolt(Voltage volts);


    /******************/
    /* Access Methods */
    /******************/
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            getDriveVelocity(), 
            Rotation2d.fromRadians(getAnglePosition().in(Radians))
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            getDrivePosition(), 
            Rotation2d.fromRadians(getAnglePosition().in(Radians))
        );
    }

    public abstract Distance getDrivePosition();
    public abstract LinearVelocity getDriveVelocity();
    public abstract Angle getAnglePosition();
    public abstract AngularVelocity getAngleVelocity();
}
