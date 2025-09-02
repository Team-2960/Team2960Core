package frc.lib2960.subsystems.drivetrain;

import frc.lib2960.config.basic.PIDConfig;

public interface HolonomicDrivetrain extends Drivetrain{
    public PIDConfig getLinearPathPlannerPID();
    public PIDConfig getAngularPathPlannerPID();
}
