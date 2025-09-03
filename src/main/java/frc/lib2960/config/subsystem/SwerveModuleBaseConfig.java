package frc.lib2960.config.subsystem;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class SwerveModuleBaseConfig {
    /** Swerve module name */
    public String name;

    /** Drive motor ID. */
    public int driveMotorID;
    /** Angle motor ID. */
    public int angleMotorID;
    /** Angle Encoder ID */
    public int angleEncoderID;

    /** Module x offset from robot origin. */
    public Distance xPos;
    /** Module y offset from robot origin. */
    public Distance yPos;

    /** Invert drive motor flag. Defaults to false. */
    public boolean invertDriveMotor = false;
    /** Invert angle motor flag. Defaults to false. */
    public boolean invertAngleMotor = false;
    /** Invert angle encoder flag. Defaults to false. */
    public boolean invertAngleEncoder = false;

    /** Angle encoder offset. Defaults to zeto rotations. */
    public Angle angleEncoderOffset = Rotations.zero();

    /**
     * Constructor
     * 
     * @param name           Name of the module
     * @param driveMotorID   drive motor ID
     * @param angleMotorID   angle motor ID
     * @param angleEncoderID angle encoder ID
     * @param xPos           Module x offset from robot origin.
     * @param yPos           Module y offset from robot origin.
     */
    public SwerveModuleBaseConfig(
            String name,
            int driveMotorID,
            int angleMotorID,
            int angleEncoderID,
            Distance xPos,
            Distance yPos) {

        this.name = name;
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.angleEncoderID = angleEncoderID;
        this.xPos = xPos;
        this.yPos = yPos;
    }

    /**
     * Sets the drive motor inverted flag. Defaults to false.
     * 
     * @param invert invert flag
     * @return current configuration object
     */
    public SwerveModuleBaseConfig setInvertDriveMotor(boolean invert) {
        this.invertDriveMotor = invert;
        return this;
    }

    /**
     * Sets the angle motor inverted flag. Defaults to false.
     * 
     * @param invert invert flag
     * @return current configuration object
     */
    public SwerveModuleBaseConfig setInvertAngleMotor(boolean invert) {
        this.invertAngleMotor = invert;
        return this;
    }

    /**
     * Sets the angle encoder inverted flag. Defaults to false.
     * 
     * @param invert invert flag
     * @return current configuration object
     */
    public SwerveModuleBaseConfig setInvertAngleEncoder(boolean invert) {
        this.invertAngleEncoder = invert;
        return this;
    }

    /**
     * Sets the angle encoder offset. Defaults to zero rotations.
     * 
     * @param offset angle encoder offset
     * @return current configuration object
     */
    public SwerveModuleBaseConfig setAngleEncoderOffset(Angle offset) {
        this.angleEncoderOffset = offset;
        return this;
    }

    public Translation2d getTranslation() {
        return new Translation2d(xPos.in(Meters), yPos.in(Meters));
    }
}
