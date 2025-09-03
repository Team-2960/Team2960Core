package frc.lib2960.subsystem.drivetrain.swerve;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import frc.lib2960.controller.AngularController;
import frc.lib2960.controller.LinearController;
import frc.lib2960.config.subsystem.SwerveModuleBaseConfig;
import frc.lib2960.config.subsystem.SwerveModuleCommonConfig;
import frc.lib2960.helper.AngleUtil;

public abstract class SwerveModuleBase {
    // TODO Implement SysID
    // TODO Implement Telemetry
    // TODO Implement Logging

    /**********************/
    /* Config Variables */
    /**********************/
    protected final SwerveModuleCommonConfig commonConfig;
    protected final SwerveModuleBaseConfig config;

    /*************************/
    /* Calculation Variables */
    /*************************/
    private final MutLinearVelocity driveCurVel = MetersPerSecond.mutable(0);
    private final MutLinearVelocity driveTarget = MetersPerSecond.mutable(0);
    private final MutVoltage driveVoltCalc = Volts.mutable(0);

    private final MutAngle angleCurPos = Degrees.mutable(0);
    private final MutAngularVelocity angleCurVel = DegreesPerSecond.mutable(0);
    private final MutAngle angleTarget = Degrees.mutable(0);
    private final MutAngularVelocity angleVelCalc = DegreesPerSecond.mutable(0);
    private final MutVoltage angleVoltCalc = Volts.mutable(0);

    private final LinearController driveCtrl;
    private final AngularController angleCtrl;

    /****************************/
    /* Driver Station Variables */
    /****************************/

    /****************/
    /* Constructors */
    /****************/
    /**
     * Constructor
     * 
     * @param commonConfig config common to all modules
     * @param config       module specific config
     */
    public SwerveModuleBase(SwerveModuleCommonConfig commonConfig, SwerveModuleBaseConfig config) {
        this.commonConfig = commonConfig;
        this.config = config;

        this.driveCtrl = commonConfig.driveCtrlConfig.getController();
        this.angleCtrl = commonConfig.angleCtrlConfig.getController();
    }

    /*******************/
    /* Control Methods */
    /*******************/
    /**
     * Sets the target state of the module
     * 
     * @param state target state of the module
     */
    public void setState(SwerveModuleState state) {
        updateAngle(state.angle);
        updateDrive(state.speedMetersPerSecond);
    }

    /**
     * Updates the module angle control
     * 
     * @param angle target angle
     */
    public void updateAngle(Rotation2d angle) {
        // TODO Implement using units library
        // Calculate angle voltage
        getAnglePosition(angleCurPos);
        getAngleVelocity(angleCurVel);
        angleTarget.mut_replace(
                AngleUtil.nearestRotationDegrees(
                        angleCurPos.in(Degrees),
                        angle.getDegrees()),
                Degrees);

        angleCtrl.updateVelocity(angleCurPos, angleCurVel, angleTarget, angleVelCalc);
        angleCtrl.updateVoltage(angleCurPos, angleCurVel, angleVelCalc, angleVoltCalc);

        setAngleVolt(angleVoltCalc);
    }

    public void updateDrive(double metersPerSecond) {
        // Calculate drive voltage
        // TODO Implement using units library
        // TODO Include couple ratio into drive speed calculation

        getDriveVelocity(driveCurVel);
        driveTarget.mut_replace(metersPerSecond, MetersPerSecond);

        driveCtrl.updateVoltage(getDriveVelocity(), driveTarget, driveVoltCalc);
    }

    /**
     * Sets the voltage of the drive motor
     * 
     * @param volts voltage to set to the drive motor
     */
    public abstract void setDriveVolt(Voltage volts);

    /**
     * Sets the voltage of the angle motor
     * 
     * @param volts voltage to set to the angle motor
     */
    public abstract void setAngleVolt(Voltage volts);

    /******************/
    /* Access Methods */
    /******************/
    /**
     * Gets the current state of the swerve module
     * 
     * @return current state of the swerve module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                getDriveVelocity(),
                Rotation2d.fromDegrees(getAnglePosition().in(Degrees)));
    }

    /**
     * Gets the current position of the swerve module
     * 
     * @return current position of the swerve module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getDrivePosition(),
                Rotation2d.fromDegrees(getAnglePosition().in(Degrees)));
    }

    /**
     * Gets the current drive position
     * 
     * @return current drive position
     */
    public Distance getDrivePosition() {
        var result = Meters.mutable(0);
        getDrivePosition(result);
        return result;
    }

    /**
     * Gets the current drive velocity
     * 
     * @return current drive velocity
     */
    public LinearVelocity getDriveVelocity() {
        var result = MetersPerSecond.mutable(0);
        getDriveVelocity(result);
        return result;
    }

    /**
     * Gets the current angle position
     * 
     * @return current angle position
     */
    public Angle getAnglePosition() {
        var result = Degrees.mutable(0);
        getAnglePosition(result);
        return result;
    }

    /**
     * Gets the current angle velocity
     * 
     * @return current angle velocity
     */
    public AngularVelocity getAngleVelocity() {
        var result = DegreesPerSecond.mutable(0);
        getAngleVelocity(result);
        return result;
    }

    /**
     * Gets the current drive position
     * 
     * @param result mutable object to store the result
     */
    public abstract void getDrivePosition(MutDistance result);

    /**
     * Gets the current drive velocity
     * 
     * @param result mutable object to store the result
     */
    public abstract void getDriveVelocity(MutLinearVelocity result);

    /**
     * Gets the current drive motor applied voltage
     * 
     * @param result
     */
    public abstract void getDriveVoltage(MutVoltage result);

    /**
     * Gets the current angle position
     * 
     * @param result mutable object to store the result
     */
    public abstract void getAnglePosition(MutAngle result);

    /**
     * Gets the current angle velocity
     * 
     * @param result mutable object to store the result
     */
    public abstract void getAngleVelocity(MutAngularVelocity result);

    /**
     * Gets the current angle motor applied voltage
     * 
     * @param result
     */
    public abstract void getAngleVoltage(MutVoltage result);

    /*************************/
    /* Static Helper Methods */
    /*************************/

    /**
     * Gets a list of translations from a list of modules
     * 
     * @param modules list of modules
     * @return list of translations
     */
    public static Translation2d[] getTranslations(SwerveModuleBase[] modules) {
        Translation2d[] translations = new Translation2d[modules.length];

        for (int i = 0; i < modules.length; i++)
            translations[i] = modules[i].config.getTranslation();

        return translations;
    }

    /**
     * Gets a list of module positions from a list of modules
     * 
     * @param modules list of modules
     * @return list of positions
     */
    public static SwerveModulePosition[] getPositions(SwerveModuleBase[] modules) {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];

        for (int i = 0; i < modules.length; i++)
            positions[i] = modules[i].getPosition();

        return positions;
    }

    /**
     * Gets a list of module states from a list of modules
     * 
     * @param modules list of modules
     * @return list of states
     */
    public static SwerveModuleState[] getStates(SwerveModuleBase[] modules) {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];

        for (int i = 0; i < modules.length; i++)
            states[i] = modules[i].getState();

        return states;
    }

}
