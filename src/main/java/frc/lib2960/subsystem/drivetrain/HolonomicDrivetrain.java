package frc.lib2960.subsystem.drivetrain;

import frc.lib2960.controller.PIDConfig;

public interface HolonomicDrivetrain extends Drivetrain {
    /**
     * Gets the Linear PID parameters for PathPlanner
     * 
     * @return Linear PID parameters for PathPlanner
     */
    public PIDConfig getLinearPathPlannerPID();

    /**
     * Gets the Angular PID parameters for PathPlanner
     * 
     * @return Angular PID parameters for PathPlanner
     */
    public PIDConfig getAngularPathPlannerPID();
}
