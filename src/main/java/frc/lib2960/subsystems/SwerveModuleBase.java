package frc.lib2960.subsystems;

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
import frc.lib2960.controllers.AngularController;
import frc.lib2960.controllers.LinearController;
import frc.lib2960.settings.SwerveModuleBaseSettings;
import frc.lib2960.settings.SwerveModuleCommonSettings;
import frc.lib2960.util.AngleUtil;

public abstract class SwerveModuleBase {
    /**********************/
    /* Settings Variables */
    /**********************/
    protected final SwerveModuleCommonSettings commonSettings;
    protected final SwerveModuleBaseSettings settings;

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
     * @param commonSettings settings common to all modules
     * @param settings       module specific settings
     */
    public SwerveModuleBase(SwerveModuleCommonSettings commonSettings, SwerveModuleBaseSettings settings) {
        this.commonSettings = commonSettings;
        this.settings = settings;

        this.driveCtrl = commonSettings.driveCtrlSettings.getController();
        this.angleCtrl = commonSettings.angleCtrlSettings.getController();
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
        // Calculate angle voltage
        getAnglePosition(angleCurPos);
        getAngleVelocity(angleCurVel);
        angleTarget.mut_replace(
                AngleUtil.nearestRotationDegrees(
                        angleCurPos.in(Degrees),
                        state.angle.getDegrees()),
                Degrees);

        angleCtrl.updateVelocity(angleCurPos, angleCurVel, angleTarget, angleVelCalc);
        angleCtrl.updateVoltage(angleCurPos, angleCurVel, angleVelCalc, angleVoltCalc);

        setAngleVolt(angleVoltCalc);

        // Calculate drive voltage
        // TODO Include couple ratio into drive speed calculation

        getDriveVelocity(driveCurVel);
        driveTarget.mut_replace(state.speedMetersPerSecond, MetersPerSecond);

        driveCtrl.updateVoltage(getDriveVelocity(), driveTarget, driveVoltCalc);

        setDriveVolt(driveVoltCalc);
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
            translations[i] = modules[i].settings.getTranslation();

        return translations;
    }

    /**
     * Gets a list of module positions from a list of modules
     * @param modules   list of modules
     * @return  list of positions
     */
    public static SwerveModulePosition[] getPositions(SwerveModuleBase[] modules) {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];

        for (int i = 0; i < modules.length; i++)
            positions[i] = modules[i].getPosition();

        return positions;
    }

    /**
     * Gets a list of module states from a list of modules
     * @param modules   list of modules
     * @return  list of states
     */
    public static SwerveModuleState[] getStates(SwerveModuleBase[] modules) {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];

        for (int i = 0; i < modules.length; i++)
            states[i] = modules[i].getState();

        return states;
    }


}
