package frc.lib2960.subsystem.drivetrain.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import frc.lib2960.config.subsystem.SwerveModuleBaseConfig;
import frc.lib2960.config.subsystem.SwerveModuleCommonConfig;

public class RevFlexMaxSwerveModule extends SwerveModuleBase {
    private final SparkFlex driveMotor;
    private final SparkMax angleMotor;

    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder angleAbsEncoder;
    private final RelativeEncoder angleVelEncoder;

    public RevFlexMaxSwerveModule(SwerveModuleCommonConfig commonConfig, SwerveModuleBaseConfig config) {
        super(commonConfig, config);

        driveMotor = new SparkFlex(config.driveMotorID,MotorType.kBrushless);
        angleMotor = new SparkMax(config.driveMotorID,MotorType.kBrushless);


        driveEncoder = driveMotor.getEncoder();

        angleAbsEncoder = angleMotor.getAbsoluteEncoder();
        angleVelEncoder = angleMotor.getAlternateEncoder();
    }

    @Override
    public void setDriveVolt(Voltage volts) {
        driveMotor.setVoltage(volts.in(Volts));
    }

    @Override
    public void setAngleVolt(Voltage volts) {
        angleMotor.setVoltage(volts.in(Volts));
    }

    @Override
    public void getDrivePosition(MutDistance result) {
        result
        .mut_replace(commonConfig.wheelCircumference)
        .mut_times(commonConfig.driveRatio)
        .mut_times(driveEncoder.getPosition());
    }

    @Override
    public void getDriveVelocity(MutLinearVelocity result) {
        result
        .mut_replace(commonConfig.wheelCircumference.in(Meters), MetersPerSecond)
        .mut_times(commonConfig.driveRatio)
        .mut_times(driveEncoder.getVelocity());
    }

    @Override
    public void getDriveVoltage(MutVoltage result) {
        result.mut_replace(driveMotor.getAppliedOutput() * driveMotor.getBusVoltage(), Volts);
    }

    @Override
    public void getAnglePosition(MutAngle result) {
        result.mut_replace(angleAbsEncoder.getPosition(), Rotations);
    }

    @Override
    public void getAngleVelocity(MutAngularVelocity result) {
        result.mut_replace(angleVelEncoder.getVelocity(), RotationsPerSecond);
    }

    @Override
    public void getAngleVoltage(MutVoltage result) {
        result.mut_replace(angleMotor.getAppliedOutput() * angleMotor.getBusVoltage(), Volts);
    }


}
