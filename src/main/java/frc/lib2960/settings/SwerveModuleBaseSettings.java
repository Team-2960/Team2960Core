package frc.lib2960.settings;

import static edu.wpi.first.units.Units.Rotations;

import java.util.Optional;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class SwerveModuleBaseSettings {
    public final String name;

    public final int driveMotorID;
    public final int angleMotorID;
    public final int angleEncoderID;

    public final Distance xPos;
    public final Distance yPos;

    public boolean invertDriveMotor = false;
    public boolean invertAngleMotor = false;
    public boolean invertAngleEncoder = false;

    public Angle encoderOffset = Rotations.zero();

    public Optional<AngularVelocity> maxAngleVel = Optional.empty();
    public Optional<AngularAcceleration> maxAngleAccel = Optional.empty();

    public SwerveModuleBaseSettings(
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
}
